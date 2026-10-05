#!/usr/bin/env python3

"""
把 AMCL 保存的 CSV 轨迹叠加到 Nav2 地图图片上.

用途：
    在不依赖 RViz 的情况下，直观比较 AMCL 估计轨迹与操控机器人时
    实际经过的地图区域。脚本读取地图 PGM、同名 Nav2 YAML 元数据和
    amcl_path_publisher 保存的 CSV，将轨迹线、轨迹点、起点与终点画到
    地图上，并在工作区根目录的 test_outputs/ 中创建本次运行的子目录。

用法：
    python3 scripts/plot_amcl_trajectory.py \
        maps/test_env.pgm \
        trajectories/amcl_acceptance_01.csv

    默认读取与 PGM 同目录、同文件名的 YAML，例如 test_env.pgm 对应
    test_env.yaml。也可通过 --map-yaml 显式指定 YAML，通过 --scale 调整
    输出图片放大倍数。运行完成后，终端会打印生成图片的绝对路径。

CSV 至少需要包含 x、y、yaw 列；本项目生成的完整表头为：
    index,stamp_sec,stamp_nanosec,frame_id,x,y,yaw

依赖：python3-pil、python3-yaml。
"""

import argparse
import csv
import math
import re
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

try:
    import yaml
    from PIL import Image, ImageDraw
except ImportError as error:
    raise SystemExit(
        "缺少绘图依赖，请安装 python3-pil 和 python3-yaml："
        " sudo apt install python3-pil python3-yaml"
    ) from error


@dataclass(frozen=True)
class TrajectoryPoint:
    """One AMCL pose in the map coordinate frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MapMetadata:
    """Nav2 map geometry needed for world-to-image conversion."""

    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float


def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="把 AMCL CSV 轨迹叠加到 Nav2 PGM 地图上。"
    )
    parser.add_argument("map_pgm", type=Path, help="Nav2 地图 PGM 文件路径")
    parser.add_argument("trajectory_csv", type=Path, help="AMCL 轨迹 CSV 文件路径")
    parser.add_argument(
        "--map-yaml",
        type=Path,
        help="Nav2 地图 YAML；省略时自动查找与 PGM 同名的 .yaml/.yml",
    )
    parser.add_argument(
        "--scale",
        type=int,
        default=4,
        help="输出图片相对原始地图的放大倍数，默认 4",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "test_outputs",
        help="输出根目录，默认是工作区根目录下的 test_outputs",
    )
    return parser.parse_args()


def resolve_input_file(path, description):
    """Resolve and validate one input file."""
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise ValueError(f"{description}不存在或不是文件：{resolved}")
    return resolved


def find_map_yaml(map_pgm, requested_yaml):
    """Find the Nav2 YAML stored beside the supplied PGM."""
    if requested_yaml is not None:
        return resolve_input_file(requested_yaml, "地图 YAML")

    candidates = (map_pgm.with_suffix(".yaml"), map_pgm.with_suffix(".yml"))
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()

    raise ValueError(
        "没有找到地图元数据。请把 YAML 放在 PGM 旁并使用相同文件名，"
        "或通过 --map-yaml 指定。"
    )


def load_map_metadata(map_yaml):
    """Load resolution and origin from a Nav2 map YAML."""
    try:
        with map_yaml.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"无法读取地图 YAML {map_yaml}：{error}") from error

    if not isinstance(data, dict):
        raise ValueError(f"地图 YAML 内容无效：{map_yaml}")

    try:
        resolution = float(data["resolution"])
        origin = data["origin"]
        origin_x, origin_y, origin_yaw = (float(value) for value in origin)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "地图 YAML 必须包含正数 resolution 和 [x, y, yaw] 格式的 origin"
        ) from error

    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("地图 resolution 必须是大于 0 的有限数值")
    values = (origin_x, origin_y, origin_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("地图 origin 必须全部是有限数值")

    return MapMetadata(resolution, origin_x, origin_y, origin_yaw)


def load_trajectory(trajectory_csv):
    """Load and validate AMCL map-frame poses from CSV."""
    points = []
    frames = set()
    try:
        with trajectory_csv.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            required_fields = {"x", "y", "yaw"}
            if reader.fieldnames is None:
                raise ValueError("轨迹 CSV 缺少表头")
            missing_fields = required_fields.difference(reader.fieldnames)
            if missing_fields:
                missing = ", ".join(sorted(missing_fields))
                raise ValueError(f"轨迹 CSV 缺少字段：{missing}")

            for line_number, row in enumerate(reader, start=2):
                try:
                    point = TrajectoryPoint(
                        x=float(row["x"]),
                        y=float(row["y"]),
                        yaw=float(row["yaw"]),
                    )
                except (TypeError, ValueError) as error:
                    raise ValueError(
                        f"轨迹 CSV 第 {line_number} 行包含无效坐标"
                    ) from error

                if not all(
                    math.isfinite(value) for value in (point.x, point.y, point.yaw)
                ):
                    raise ValueError(f"轨迹 CSV 第 {line_number} 行包含非有限数值")

                frame_id = (row.get("frame_id") or "").strip()
                if frame_id:
                    frames.add(frame_id)
                points.append(point)
    except OSError as error:
        raise ValueError(f"无法读取轨迹 CSV {trajectory_csv}：{error}") from error

    if not points:
        raise ValueError("轨迹 CSV 中没有可绘制的位姿")
    if frames and frames != {"map"}:
        frame_names = ", ".join(sorted(frames))
        raise ValueError(f"轨迹坐标系不是单一的 map，而是：{frame_names}")
    return points


def world_to_pixel(point, metadata, image_height, scale):
    """Convert a map-frame pose to scaled image pixel coordinates."""
    delta_x = point.x - metadata.origin_x
    delta_y = point.y - metadata.origin_y
    cosine = math.cos(metadata.origin_yaw)
    sine = math.sin(metadata.origin_yaw)

    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    pixel_x = local_x / metadata.resolution
    pixel_y = image_height - 1 - local_y / metadata.resolution
    return (round(pixel_x * scale), round(pixel_y * scale))


def make_output_directory(output_root, trajectory_name):
    """Create a timestamped child directory without overwriting an old run."""
    root = output_root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", trajectory_name).strip("._")
    safe_name = safe_name or "trajectory"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    for sequence in range(1000):
        suffix = "" if sequence == 0 else f"_{sequence:02d}"
        output_directory = root / f"{safe_name}_{timestamp}{suffix}"
        try:
            output_directory.mkdir()
            return output_directory
        except FileExistsError:
            continue
    raise ValueError("无法为本次绘图创建唯一的输出子目录")


def draw_trajectory(map_pgm, metadata, points, scale, output_file):
    """Render the map, path, points, start marker, and end marker."""
    if scale < 1:
        raise ValueError("--scale 必须是大于或等于 1 的整数")

    try:
        with Image.open(map_pgm) as source_image:
            map_image = source_image.convert("RGBA")
    except OSError as error:
        raise ValueError(f"无法读取地图图片 {map_pgm}：{error}") from error

    original_width, original_height = map_image.size
    output_size = (original_width * scale, original_height * scale)
    resampling = getattr(Image, "Resampling", Image)
    map_image = map_image.resize(output_size, resampling.NEAREST)

    pixels = [
        world_to_pixel(point, metadata, original_height, scale) for point in points
    ]
    inside_count = sum(
        0 <= x < output_size[0] and 0 <= y < output_size[1] for x, y in pixels
    )

    overlay = Image.new("RGBA", output_size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    line_width = max(2, scale)
    point_radius = max(2, scale)

    if len(pixels) > 1:
        draw.line(pixels, fill=(255, 30, 30, 220), width=line_width, joint="curve")
    for pixel_x, pixel_y in pixels:
        draw.ellipse(
            (
                pixel_x - point_radius,
                pixel_y - point_radius,
                pixel_x + point_radius,
                pixel_y + point_radius,
            ),
            fill=(255, 190, 0, 210),
        )

    marker_radius = max(5, scale * 2)
    start_x, start_y = pixels[0]
    end_x, end_y = pixels[-1]
    draw.ellipse(
        (
            start_x - marker_radius,
            start_y - marker_radius,
            start_x + marker_radius,
            start_y + marker_radius,
        ),
        fill=(0, 190, 70, 255),
        outline=(0, 80, 20, 255),
        width=max(1, scale // 2),
    )
    draw.ellipse(
        (
            end_x - marker_radius,
            end_y - marker_radius,
            end_x + marker_radius,
            end_y + marker_radius,
        ),
        fill=(20, 100, 255, 255),
        outline=(0, 30, 120, 255),
        width=max(1, scale // 2),
    )
    draw.text((start_x + marker_radius + 2, start_y), "START", fill=(0, 90, 20, 255))
    draw.text((end_x + marker_radius + 2, end_y), "END", fill=(0, 40, 150, 255))

    rendered = Image.alpha_composite(map_image, overlay).convert("RGB")
    rendered.save(output_file, format="PNG")
    return original_width, original_height, inside_count


def main():
    """Load inputs, render one trajectory overlay, and report its location."""
    arguments = parse_arguments()
    try:
        map_pgm = resolve_input_file(arguments.map_pgm, "地图 PGM")
        trajectory_csv = resolve_input_file(arguments.trajectory_csv, "轨迹 CSV")
        map_yaml = find_map_yaml(map_pgm, arguments.map_yaml)
        metadata = load_map_metadata(map_yaml)
        points = load_trajectory(trajectory_csv)
        output_directory = make_output_directory(
            arguments.output_root, trajectory_csv.stem
        )
        output_file = output_directory / "trajectory_overlay.png"
        width, height, inside_count = draw_trajectory(
            map_pgm, metadata, points, arguments.scale, output_file
        )
    except ValueError as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1

    print(f"地图：{map_pgm} ({width}x{height}, {metadata.resolution:g} m/pixel)")
    print(f"轨迹：{trajectory_csv} ({len(points)} 个点，{inside_count} 个位于地图内)")
    if inside_count == 0:
        print("警告：所有轨迹点都位于地图图像范围之外，请检查初始位姿和地图。")
    print(f"输出：{output_file}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
