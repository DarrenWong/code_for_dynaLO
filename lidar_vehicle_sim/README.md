# LiDAR Vehicle Simulator

Inject synthetic moving vehicles into a static LiDAR point-cloud scene and
produce a ground-truth-labelled ROS bag — without any physical sensor.

https://github.com/user-attachments/assets/video_demo.mp4

> Demo: two double-deck buses (right and left lanes) moving through a static
> scene recorded with a Velodyne HDL-64.

---

## How it works

Each simulated vehicle is a hollow bounding-box mesh sampled at 10 pts/m.
For every incoming LiDAR frame the script:

1. **Moves** the vehicle models along the Y axis (looping back after 20 m).
2. **Culls** background points whose azimuth and elevation fall inside the
   vehicle's angular footprint (occlusion simulation).
3. **Merges** vehicle points into the filtered cloud.
4. **Publishes** the result on `/points_simulated` and writes it to
   `dynamic_sim.bag`.

Points are tagged in the `time` field: `10` = dynamic, `5` = static, which
lets downstream algorithms train or evaluate on known ground truth.

---

## Repository layout

```
.
├── dynamic_points_creation.py                 # Vehicle class — box model & transforms
├── pointcloud_vehicle_sim_velocity_v3_both_clean.py  # Main simulator node
├── non_dynamic.bag                            # Input: static scene recording
├── rviz_for_vis.rviz                          # RViz config for visualisation
├── dynamic_stat.txt                           # Output: per-frame dynamic-point stats
└── video_demo.mp4                             # Demo recording
```

---

## Requirements

| Dependency | Version |
|---|---|
| Ubuntu | 20.04 |
| ROS | Noetic |
| Python | 3.8+ |
| numpy | any recent |
| scipy | ≥ 1.4 |
| ros-numpy | via apt |
| pynput | via pip |

---

## Installation

```bash
# ROS system packages
sudo apt update
sudo apt install -y ros-noetic-ros-numpy

# Python packages
pip install pynput
```

Source ROS before running:

```bash
source /opt/ros/noetic/setup.bash
```

---

## Quick start

```bash
# 1. Clone / enter the repository
cd LiDAR_vehicle_sim_test_for_AI

# 2. Run the simulator
python3 pointcloud_vehicle_sim_velocity_v3_both_clean.py
```

The script first replays `non_dynamic.bag` frame-by-frame (bag mode), then
keeps the ROS node alive for live Velodyne data (live mode).

---

## Keyboard controls (live mode)

After bag processing completes the node stays up and listens to
`/velodyne_points`.  Use these keys to toggle additional simulated objects
(extend `pointcloud_callback` with your own point clouds):

| Key | Flag toggled |
|-----|---|
| `w` | `sim_front` |
| `a` | `sim_left` |
| `d` | `sim_right` |
| `s` | `sim_rear` |
| `e` | `sim_right_top` |
| `c` | `sim_right_bottom` |

---

## Output

| File | Description |
|------|---|
| `dynamic_sim.bag` | Augmented frames on `/simulated_pointcloud` |
| `dynamic_stat.txt` | Per-frame dynamic/static/total point counts and dynamic ratio |

Inspect the output bag:

```bash
rosbag info dynamic_sim.bag
```

---

## Visualisation with RViz

```bash
# Terminal 1 — play the output bag on loop
rosbag play --loop dynamic_sim.bag

# Terminal 2 — launch RViz with the provided config
rviz -d rviz_for_vis.rviz
```

The RViz config displays `/points_simulated` coloured by the `time` field so
dynamic points (value 10) appear red and static points (value 0) appear white.

---

## Customising vehicle size and position

Edit the `Vehicle` initialisations near the top of the main script:

```python
# Vehicle(width_m, length_m, height_m, [x, y, z])
dynamic_bus_right = Vehicle(2.5, 12.8, 3.0, [4.0, -4.0, 0.0])
dynamic_bus_left  = Vehicle(2.5, 12.8, 3.0, [-4.0, -6.0, 0.0])
```

Adjust `move(speed)` calls in `pointcloud_callback` to change vehicle speed.

---

## Troubleshooting

**`ModuleNotFoundError: No module named 'ros_numpy'`**
```bash
sudo apt install ros-noetic-ros-numpy
```

**`scipy.spatial.transform.Rotation has no attribute 'as_dcm'`**\
Upgrade scipy to ≥ 1.4 — `as_dcm()` was removed; the code now uses
`as_matrix()`.

**`rosbag.bag.ROSBagException` on non_dynamic.bag**\
Make sure the bag file is present in the working directory:
```bash
ls -lh non_dynamic.bag
```

**Slow point-cloud creation on first run**\
The `Vehicle.__init__` loop is intentionally simple; for large vehicles or
high resolution consider pre-generating and caching the box mesh.

---

## License

MIT License — see [LICENSE](LICENSE).
