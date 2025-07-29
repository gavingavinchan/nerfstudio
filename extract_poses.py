import argparse
import json
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


def get_video_fps(video_path: Path) -> float:
    """Retrieves the frame rate (FPS) of a video file using ffprobe."""
    if not video_path.is_file():
        print(f"Error: Video file not found at {video_path}", file=sys.stderr)
        sys.exit(1)
    command = [
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=r_frame_rate", "-of",
        "default=noprint_wrappers=1:nokey=1", str(video_path)
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        num, den = map(int, result.stdout.strip().split('/'))
        return num / den if den != 0 else 0
    except (subprocess.CalledProcessError, ValueError, FileNotFoundError) as e:
        print(f"Error getting FPS: {e}. Ensure FFmpeg is installed.", file=sys.stderr)
        sys.exit(1)


def extract_frame_number(file_path: str) -> int:
    """Extracts the frame number from a file path like 'images/frame_0001.png'."""
    match = re.search(r"frame_(\d+)", file_path)
    if not match:
        raise ValueError(f"Could not extract frame number from {file_path}")
    return int(match.group(1))


def main():
    """Main function to extract 6DoF data and save it in ROS2-compatible YAML."""
    parser = argparse.ArgumentParser(
        description="Extract 6DoF camera poses from Nerfstudio and save as ROS2-compatible YAML."
    )
    parser.add_argument(
        "--project_path", type=Path, required=True,
        help="Path to the Nerfstudio project directory (e.g., 'outputs/bigScrew2')."
    )
    parser.add_argument(
        "--video_path", type=Path, required=True,
        help="Path to the original video file."
    )
    parser.add_argument(
        "--output_dir", type=Path, required=True,
        help="Directory where the final YAML file will be saved."
    )
    parser.add_argument(
        "--frame_id", type=str, default="map",
        help="The fixed world frame_id for the pose header. Defaults to 'map'."
    )
    args = parser.parse_args()

    # --- Define the coordinate system transformation ---
    # COLMAP/Nerfstudio (OpenCV convention): +X right, +Y down, +Z forward
    # ROS (REP-103): +X forward, +Y left, +Z up
    # This matrix converts a point from OpenCV to ROS coordinates
    CV_TO_ROS_FRAME = np.array([
        [0, 0, 1, 0],
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ])

    transforms_path = args.project_path / "transforms.json"
    if not transforms_path.is_file():
        print(f"Error: 'transforms.json' not found in {args.project_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Reading transforms from: {transforms_path}")
    with open(transforms_path, 'r') as f:
        transforms_data = json.load(f)

    print(f"Getting FPS from: {args.video_path}")
    fps = get_video_fps(args.video_path)
    print(f"Detected FPS: {fps}")

    all_poses = []
    print("Processing frames and converting to ROS coordinate system...")
    for frame in sorted(transforms_data.get("frames", []), key=lambda f: f['file_path']):
        try:
            frame_number = extract_frame_number(frame["file_path"])
            timestamp_sec_float = (frame_number - 1) / fps if fps > 0 else 0
            
            ts_sec, ts_nanosec_float = divmod(timestamp_sec_float, 1)
            ts_nanosec = int(ts_nanosec_float * 1e9)

            c2w_colmap = np.array(frame["transform_matrix"])
            c2w_ros = CV_TO_ROS_FRAME @ c2w_colmap

            position = c2w_ros[:3, 3]
            rotation_matrix = c2w_ros[:3, :3]
            quat = R.from_matrix(rotation_matrix).as_quat()

            pose_stamped = {
                'header': {
                    'stamp': {'sec': int(ts_sec), 'nanosec': ts_nanosec},
                    'frame_id': args.frame_id
                },
                'pose': {
                    'position': {'x': float(position[0]), 'y': float(position[1]), 'z': float(position[2])},
                    'orientation': {'x': float(quat[0]), 'y': float(quat[1]), 'z': float(quat[2]), 'w': float(quat[3])}
                }
            }
            all_poses.append(pose_stamped)

        except (ValueError, KeyError) as e:
            print(f"Skipping frame due to error: {e}", file=sys.stderr)
            continue

    if not all_poses:
        print("Warning: No poses were extracted.", file=sys.stderr)
        sys.exit(0)

    video_name = args.video_path.stem
    output_file = args.output_dir / f"COLMAP_camera_pose_{video_name}.yaml"
    args.output_dir.mkdir(parents=True, exist_ok=True)

    yaml_header = (
        "# ROS2-compatible 6DoF camera poses extracted from Nerfstudio (COLMAP).\n"
        "# Each entry represents a geometry_msgs/PoseStamped message.\n"
        f"# The pose describes the camera's frame (camera_link) relative to the '{args.frame_id}' frame.\n"
        "# Coordinate system has been transformed to the ROS standard (X forward, Y left, Z up).\n"
    )
    
    print(f"Saving {len(all_poses)} poses to: {output_file}")
    with open(output_file, 'w') as f:
        f.write(yaml_header)
        yaml.dump({'poses': all_poses}, f, indent=2, sort_keys=False, default_flow_style=False)

    print(f"🎉 Successfully saved poses to {output_file}")

if __name__ == "__main__":
    main()