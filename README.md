# ida_camera

ROS2 camera processing node for the **Logitech C920 Pro**, feeding a **YOLOv11 buoy detector** running on a Jetson Nano.

Corrects lens distortion and enhances image quality before publishing two output streams: a full-resolution debug image and a letterboxed RGB image ready for inference.

---

## Pipeline

```
C920 raw frame (1920×1080 BGR)
  │
  ├─ Stage 1  Undistort      cv::remap with precomputed Brown-Conrady maps
  ├─ Stage 2  Denoise        Bilateral filter — edge-preserving (off by default)
  ├─ Stage 3  CLAHE          Contrast enhancement on LAB L-channel only
  ├─ Stage 4  Sharpen        Unsharp mask (off by default)
  ├─ Stage 5  Gamma          LUT-based brightness curve (off by default)
  │
  ├──▶  /camera/image_processed   BGR8  1920×1080   debug / visualisation
  └──▶  /camera/image_ml          RGB8   640×640    letterboxed → YOLOv11
```

Every stage can be toggled independently at runtime — no restart needed.

---

## Topics

| Topic | Type | Description |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Input — raw frames from the camera driver |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Input — calibration fallback if no YAML is set |
| `/camera/image_processed` | `sensor_msgs/Image` | Output — full-res BGR, all processing applied |
| `/camera/image_ml` | `sensor_msgs/Image` | Output — 640×640 RGB8, letterboxed for YOLO |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Per-stage timing, frame rate, enabled flags |

### ML topic metadata

The `frame_id` field of every `/camera/image_ml` message encodes the letterbox geometry so the YOLO node can reproject bounding boxes back to the original frame without a separate message:

```
"<original_frame_id>|s=<scale>|ox=<pad_x>|oy=<pad_y>"
```

Unproject a detection:
```python
orig_x = (box_cx - offset_x) / scale
orig_y = (box_cy - offset_y) / scale
```

---

## Parameters

All parameters are dynamically reconfigurable:
```bash
ros2 param set /camera_processor <name> <value>
```

| Parameter | Default | Description |
|---|---|---|
| `yaml_calibration_path` | `""` | Path to calibration YAML. Empty = use `camera_info` topic. |
| `undistort_alpha` | `0.0` | `0` = tight crop (no black borders), `1` = all pixels |
| `use_denoise` | `false` | Enable bilateral denoising (runs before CLAHE) |
| `denoise_d` | `9` | Bilateral filter diameter. `5` = fast, `9` = quality |
| `denoise_sigma_color` | `75.0` | Color-space sigma for bilateral filter |
| `denoise_sigma_space` | `75.0` | Spatial sigma for bilateral filter |
| `use_clahe` | `true` | Enable CLAHE contrast enhancement |
| `clahe_clip_limit` | `1.5` | Contrast cap per tile. Lower preserves buoy color fidelity |
| `clahe_tile_width` | `8` | CLAHE grid cell width in pixels |
| `clahe_tile_height` | `8` | CLAHE grid cell height in pixels |
| `use_sharpen` | `false` | Enable unsharp mask sharpening |
| `sharpen_sigma` | `1.0` | Blur sigma for unsharp mask |
| `sharpen_strength` | `0.8` | Blend weight — `0.5` subtle, `0.8` clear, `>1.2` aggressive |
| `use_gamma` | `false` | Enable gamma correction |
| `gamma` | `1.0` | `<1` brightens shadows, `>1` darkens, `2.2` linearises sRGB |
| `ml_topic` | `/camera/image_ml` | Output topic for the YOLO-ready image |
| `ml_width` | `640` | ML output width |
| `ml_height` | `640` | ML output height |
| `letterbox_fill` | `114` | Padding pixel value (YOLO standard grey — do not change unless retrained) |

---

## Build

```bash
cd ~/ros2_ws
# symlink or copy this package into src/
ln -s ~/ida_camera src/ida_camera

colcon build --packages-select ida_camera
source install/setup.bash
```

Dependencies: `rclcpp`, `rclcpp_components`, `std_msgs`, `sensor_msgs`, `diagnostic_msgs`, `cv_bridge`, OpenCV, yaml-cpp.

```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge \
                 ros-$ROS_DISTRO-camera-calibration \
                 ros-$ROS_DISTRO-v4l2-camera \
                 libopencv-dev \
                 libyaml-cpp-dev
```

---

## Run

```bash
# Start the camera driver first
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:=/dev/video0 \
    -p image_size:=[1920,1080]

# Launch the processing node
ros2 launch ida_camera camera_processor.launch.py

# With your calibration file
ros2 launch ida_camera camera_processor.launch.py \
    yaml_calibration_path:=/path/to/c920_calibration.yaml

# With sharpening and gamma enabled
ros2 launch ida_camera camera_processor.launch.py \
    use_sharpen:=true use_gamma:=true gamma:=0.8
```

---

## Calibration

Calibration is required to correct lens distortion accurately. The template values in `config/c920_calibration.yaml` are representative for the C920 at 1080p but will have residual error — calibrate your specific unit.

See **[docs/calibration_tutorial.md](docs/calibration_tutorial.md)** for a full walkthrough.

Short version:
```bash
# 1. Print an 8×6 checkerboard at 25 mm squares, tape it flat
# 2. Run the calibrator
ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.025 \
    image:=/camera/image_raw camera:=/camera

# 3. Move board until X/Y/Size/Skew bars go green (~80 samples)
# 4. Click Calibrate → Save → extract
tar xvf /tmp/calibrationdata.tar.gz -C /tmp/

# 5. Point the node at the result
ros2 launch ida_camera camera_processor.launch.py \
    yaml_calibration_path:=/tmp/ost.yaml
```

Aim for a reprojection error under **0.5 px**. Above **1.0 px** — redo with more samples covering the frame corners.

---

## Letterbox geometry (1920×1080 → 640×640)

```
scale  = min(640/1920, 640/1080) = 0.333
active = 640×360  (centred)
padding: 140 px top and bottom, fill = 114
```

Naive resize would squash the aspect ratio from 1.78 to 1.0 and hurt YOLO's bounding-box regression. Letterboxing preserves it.

---

## Jetson Nano budget (approximate, 1080p input)

| Stage | Cost |
|---|---|
| Undistort (remap) | ~4 ms |
| CLAHE | ~6 ms |
| Letterbox + BGR→RGB | ~3 ms |
| **Total (default config)** | **~13 ms — well within 33 ms at 30 fps** |
| Bilateral denoise (if enabled) | +10–15 ms |
| Unsharp mask (if enabled) | +2–4 ms |
| Gamma LUT (if enabled) | ~0 ms |

---

## License

MIT — see [LICENSE](LICENSE).
