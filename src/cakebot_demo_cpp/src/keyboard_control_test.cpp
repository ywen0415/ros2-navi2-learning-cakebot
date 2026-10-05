/*
 * keyboard_control_test.cpp
 *
 * 用途：
 *   用键盘人工控制 cakebot，验证 /cmd_vel 到 Gazebo 差速驱动插件的链路，
 *   也可用于阶段 7 的人工遥控探索和 SLAM 建图。
 *
 * 安全机制：
 *   - 只在终端处于前台时读取按键；
 *   - 持续一段时间没有收到按键时自动发布零速度；
 *   - 按下 x、空格或 q 时立即停止；
 *   - 节点退出时恢复终端设置并发布零速度。
 */
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class KeyboardControlTest : public rclcpp::Node
{
public:
  KeyboardControlTest()
  : Node("keyboard_control_test")
  {
    linear_speed_ = declare_parameter("linear_speed", 0.20);
    angular_speed_ = declare_parameter("angular_speed", 0.80);
    command_timeout_ = declare_parameter("command_timeout", 0.0);

    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error(
              "keyboard_control_test must run in an interactive terminal (stdin is not a TTY)");
    }

    if (tcgetattr(STDIN_FILENO, &original_terminal_) != 0) {
      throw std::runtime_error(
              std::string("failed to read terminal settings: ") + std::strerror(errno));
    }

    configure_terminal();
    terminal_configured_ = true;

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(50ms, [this]() {on_timer();});
    last_key_time_ = std::chrono::steady_clock::now();
    input_thread_ = std::thread([this]() {input_loop();});

    print_help();
  }

  ~KeyboardControlTest() override
  {
    stop_input_.store(true);
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
    publish_stop();
    restore_terminal();
  }

private:
  void configure_terminal()
  {
    struct termios raw_terminal = original_terminal_;
    cfmakeraw(&raw_terminal);

    // 保留 Ctrl-C 的终端信号行为，同时不回显按键。
    raw_terminal.c_lflag |= ISIG;
    raw_terminal.c_cc[VMIN] = 0;
    raw_terminal.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw_terminal) != 0) {
      throw std::runtime_error(
              std::string("failed to configure terminal: ") + std::strerror(errno));
    }
  }

  void restore_terminal()
  {
    if (terminal_configured_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
      terminal_configured_ = false;
    }
  }

  void on_timer()
  {
    if (quit_requested_.load()) {
      publish_stop();
      rclcpp::shutdown();
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    double linear_command = 0.0;
    double angular_command = 0.0;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      const double elapsed = std::chrono::duration<double>(now - last_key_time_).count();
      if (command_timeout_ > 0.0 && elapsed > command_timeout_) {
        linear_command_ = 0.0;
        angular_command_ = 0.0;
      }
      linear_command = linear_command_;
      angular_command = angular_command_;
    }

    publish_velocity(linear_command, angular_command);
  }

  void input_loop()
  {
    while (!stop_input_.load()) {
      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(STDIN_FILENO, &read_set);

      timeval timeout{};
      timeout.tv_usec = 100000;
      const int result = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
      if (result < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }
      if (result == 0 || !FD_ISSET(STDIN_FILENO, &read_set)) {
        continue;
      }

      char key = '\0';
      const ssize_t bytes_read = read(STDIN_FILENO, &key, 1);
      if (bytes_read == 1) {
        handle_key(key);
      } else if (bytes_read == 0 || (bytes_read < 0 && errno != EINTR)) {
        break;
      }
    }
  }

  void handle_key(char key)
  {
    key = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_key_time_ = std::chrono::steady_clock::now();

    switch (key) {
      case 'w':
        linear_command_ = linear_speed_;
        angular_command_ = 0.0;
        break;
      case 's':
        linear_command_ = -linear_speed_;
        angular_command_ = 0.0;
        break;
      case 'a':
        linear_command_ = 0.0;
        angular_command_ = angular_speed_;
        break;
      case 'd':
        linear_command_ = 0.0;
        angular_command_ = -angular_speed_;
        break;
      case 'x':
      case ' ':
        linear_command_ = 0.0;
        angular_command_ = 0.0;
        break;
      case 'q':
        linear_command_ = 0.0;
        angular_command_ = 0.0;
        quit_requested_.store(true);
        break;
      default:
        // 忽略未知按键，并不修改当前运动命令。
        break;
    }
  }

  void print_help() const
  {
    std::printf(
      "\nKeyboard control started. Keep this terminal focused.\n"
      "  w: forward    s: backward\n"
      "  a: turn left  d: turn right\n"
      "  x/space: stop q: quit\n"
      "Commands stay active until another key is pressed.\n"
      "Optional timeout: %.2f seconds (0 means disabled).\n\n",
      command_timeout_);
    std::fflush(stdout);
  }

  void publish_velocity(double linear_x, double angular_z)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear_x;
    command.angular.z = angular_z;
    cmd_vel_pub_->publish(command);
  }

  void publish_stop()
  {
    if (cmd_vel_pub_) {
      publish_velocity(0.0, 0.0);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread input_thread_;
  std::mutex command_mutex_;
  std::atomic_bool stop_input_{false};
  std::atomic_bool quit_requested_{false};

  termios original_terminal_{};
  bool terminal_configured_{false};

  std::chrono::steady_clock::time_point last_key_time_{};
  double linear_command_{0.0};
  double angular_command_{0.0};
  double linear_speed_{0.20};
  double angular_speed_{0.80};
  double command_timeout_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<KeyboardControlTest>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "keyboard_control_test failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
