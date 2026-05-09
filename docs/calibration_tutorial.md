# Camera Calibration Tutorial — Logitech C920 Pro

## Why calibrate?

Every physical lens introduces distortion that no two units share exactly.
Running the calibration tool once produces the K matrix and distortion coefficients
specific to *your* camera unit, at *your* chosen resolution.
The template values in `c920_calibration.yaml` are a reasonable stand-in but
will leave residual error. Calibrating takes about 10 minutes and is worth it.

---

## What you need

| Item | Notes |
|---|---|
| Printed checkerboard | 8×6 inner corners (9 columns × 7 rows of squares). Print on plain A4/Letter, **do not scale to fit** — keep square size exact. |
| Flat backing | Tape the print to a clipboard, book cover, or sheet of foam board. It must stay perfectly flat. |
| Tape measure | To measure the printed square size in metres. |
| A well-lit room | Avoid backlighting; make sure the board is evenly lit with no glare on the paper. |

**Checkerboard generator:**
```
https://calib.io/pages/camera-calibration-pattern-generator
```
Use: Pattern Type = Checkerboard, Rows = 7, Columns = 9, Checker Width = 25 mm.
Download as PDF, print at 100% (no scaling). Measure one square with a ruler —
it should be as close to 25.0 mm as possible.

---

## Step 1 — Install dependencies

```bash
sudo apt install ros-$ROS_DISTRO-camera-calibration \
                 ros-$ROS_DISTRO-v4l2-camera
```

`camera-calibration` is the calibration GUI.
`v4l2-camera` is the driver that publishes raw frames from the C920.

---

## Step 2 — Start the camera driver

Open **Terminal A**. Plug the C920 in, then:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

ros2 run v4l2_camera v4l2_camera_node \
    --ros-args \
    -p video_device:="/dev/video0" \
    -p image_size:=[1920,1080] \
    -p pixel_format:="YUYV"
```

Verify the camera is streaming:
```bash
ros2 topic hz /camera/image_raw
# Should print ~30 Hz
```

> **Note:** If your camera appears as `/dev/video2` or similar, adjust `video_device`.
> Run `ls /dev/video*` before and after plugging in to find the right device.

---

## Step 3 — Run the calibration tool

Open **Terminal B**:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.025 \
    image:=/camera/image_raw \
    camera:=/camera
```

| Argument | Meaning |
|---|---|
| `--size 8x6` | Inner corner count: 8 columns × 6 rows of *corners* (one less than squares in each direction) |
| `--square 0.025` | Square side length **in metres** — change this if you printed a different size |
| `image:=...` | Remap the tool's image subscription to our camera topic |
| `camera:=...` | Namespace for the camera_info service |

A live window opens showing the camera feed.

---

## Step 4 — Move the board until all bars go green

The GUI shows four progress bars:

| Bar | What it measures | How to fill it |
|---|---|---|
| **X** | Horizontal spread of board positions | Move board left and right across the full frame |
| **Y** | Vertical spread | Move board up and down |
| **Size** | Distance variation (zoom) | Move board close (fill ~60% of frame) then far (fill ~20%) |
| **Skew** | Tilt angle variation | Tilt the board ±30° in all axes — top-left corner close, then top-right corner close, etc. |

**Tips:**
- Move *slowly* — the tool captures a sample only when the board is held still enough.
- Cover the full field of view, especially the corners and edges.
- Minimum ~50 samples before `Calibrate` becomes active. Aim for 80–100 for a clean result.
- The board outline turns from red → green when a sample is captured.

---

## Step 5 — Calibrate and save

1. Click **Calibrate** — this takes 10–30 seconds.
2. The reprojection error is printed in the terminal.
   - Under **0.5 px** → excellent
   - 0.5–1.0 px → good, usable
   - Over 1.0 px → redo the capture (more samples at the corners / more skew variation)
3. Click **Save**. A file appears at:
   ```
   /tmp/calibrationdata.tar.gz
   ```
4. Extract it:
   ```bash
   cd /tmp
   tar xvf calibrationdata.tar.gz
   # Produces: ost.yaml  (and images/)
   ```

---

## Step 6 — Copy values into our config

Open `/tmp/ost.yaml` and copy the values into
[config/c920_calibration.yaml](../config/c920_calibration.yaml):

```bash
# Quick way — just replace the whole file:
cp /tmp/ost.yaml /path/to/ida_camera/config/c920_calibration.yaml
```

The formats are compatible. The node reads the same fields
(`camera_matrix`, `distortion_coefficients`, `image_width`, `image_height`).

---

## Step 7 — Tell the node to use it

In [config/camera_params.yaml](../config/camera_params.yaml), set:

```yaml
yaml_calibration_path: "/absolute/path/to/ida_camera/config/c920_calibration.yaml"
```

Or pass it at launch time:

```bash
ros2 launch ida_camera camera_processor.launch.py \
    yaml_calibration_path:=/home/yalin/ros2_ws/src/ida_camera/config/c920_calibration.yaml
```

---

## Step 8 — Verify the result

Start the full stack and visualise both topics in RViz2 or rqt_image_view:

```bash
# Terminal A — camera driver (already running)

# Terminal B — our processing node
ros2 launch ida_camera camera_processor.launch.py \
    yaml_calibration_path:=/path/to/c920_calibration.yaml

# Terminal C — side-by-side comparison
ros2 run rqt_image_view rqt_image_view
```

Switch between `/camera/image_raw` and `/camera/image_processed`.
Lines at the image edges that used to curve inward/outward should now appear straight.

---

## Troubleshooting

| Problem | Likely cause | Fix |
|---|---|---|
| `Could not open /dev/video0` | Wrong device path | Check `ls /dev/video*` |
| Progress bars not filling | Board moves too fast / not enough corner variation | Slow down; cover all four corners of the frame |
| Reprojection error > 1.5 px | Blurry print, warped board, bad samples | Reprint on stiffer backing; discard blurry captures |
| Black triangles at frame corners after undistortion | `undistort_alpha` is not 0 | Set `undistort_alpha: 0.0` in camera_params.yaml |
| Node logs "No calibration available" | YAML path wrong or file not found | Use an absolute path; check with `ls /your/path/c920_calibration.yaml` |
