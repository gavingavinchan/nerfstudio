### Final Plan: Extract 6DoF Camera Poses for ROS2

**Objective:** Create a Python script to generate a ROS2-compatible YAML file from `ns-process-data` output. The script will perform the necessary coordinate-system transformations to ensure the poses are correct for direct use in ROS.

**1. ROS2-Compliant Output**

*   **Format:** A YAML file (`.yaml`) containing a list of poses.
*   **Naming Convention:** The output file will be named `COLMAP_camera_pose_VIDEO_NAME.yaml`.
*   **Data Structure (`geometry_msgs/PoseStamped`):**
    *   `header`:
        *   `stamp`: A `builtin_interfaces/Time` message (`sec`, `nanosec`).
        *   `frame_id`: The name of the fixed world frame in which the pose is expressed. Defaults to **`"map"`**. This is the standard ROS convention for a global, non-moving reference frame.
    *   `pose`:
        *   `position`: The `(x, y, z)` position of the camera.
        *   `orientation`: A quaternion `(x, y, z, w)` representing the camera's orientation.
*   **Clarification:** The generated YAML file will contain a comment explaining that each pose represents the position and orientation of the camera's frame (conceptually, the `camera_link`) relative to the fixed `map` frame.

**2. Coordinate System Transformation**

*   **Problem:** COLMAP/NeRF use a computer vision coordinate system (`+X` right, `+Y` down, `+Z` forward), which is incompatible with the ROS standard (`+X` forward, `+Y` left, `+Z` up).
*   **Solution:** The script will apply a rotation to every pose to transform it from the COLMAP world frame to the ROS world frame. This is a critical step for ensuring the data is usable in tools like RViz.

**3. Develop the Python Script**

*   **Dependencies:** `PyYAML`, `numpy`, `scipy`. A note will be included to install `PyYAML`.

*   **Inputs:**
    1.  `--project_path`: Path to the `nerfstudio` project directory.
    2.  `--video_path`: Path to the original video file.
    3.  `--output_dir`: Directory for the final YAML file.
    4.  `--frame_id`: (Optional) The name for the fixed world frame. **Defaults to `"map"`**.

*   **Process:**
    1.  Load the `transforms.json` file.
    2.  Use `ffprobe` to get the video's FPS for accurate timestamps.
    3.  Define the constant rotation matrix for converting from the COLMAP to the ROS coordinate system.
    4.  Initialize a list to hold the final ROS-compliant pose data.
    5.  For each frame in the JSON data:
        *   Calculate the `sec` and `nanosec` timestamp.
        *   Extract the 4x4 `transform_matrix`.
        *   Apply the coordinate transformation to the pose.
        *   Extract the final position and quaternion from the transformed pose.
        *   Assemble the data into a nested dictionary matching the `PoseStamped` structure with `frame_id: "map"`.
    6.  Use `PyYAML` to write the list of poses to the final YAML file, including the explanatory comment at the top.

**4. Execution and Verification**

*   The script will be run from the command line. The resulting YAML file is ready to be parsed by a ROS2 node.

---

### Project Summary and Completion

**Status: Completed**
**Date: Tuesday, July 29, 2025**

All objectives outlined in the final plan have been successfully met.

*   The script `extract_poses.py` was created and refined to produce a clean, standard YAML file.
*   The script correctly implements the crucial coordinate system transformation from the COLMAP/NeRF standard to the ROS standard.
*   The final output file, `COLMAP_camera_pose_session_2025-07-24_21-51-15.yaml`, was generated and verified. It is correctly formatted and ready for direct use within a ROS2 environment.

**Example Execution Command:**
```sh
python /home/gavin/nerfstudio/extract_poses.py \
    --project_path /home/gavin/nerfstudio/outputs/bigScrew2 \
    --video_path /home/gavin/nerfstudio/data/nerfstudio/bigScrew2/session_2025-07-24_21-51-15.avi \
    --output_dir /home/gavin/nerfstudio/outputs/bigScrew2
```