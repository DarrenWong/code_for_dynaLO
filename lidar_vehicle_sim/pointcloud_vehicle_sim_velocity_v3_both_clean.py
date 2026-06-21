#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""LiDAR vehicle simulator — injects dynamic vehicle point clouds into a
static scene bag and writes the augmented frames to a new bag file.

Usage:
    python3 pointcloud_vehicle_sim_velocity_v3_both_clean.py

The script has two modes that run in sequence:
  1. Bag mode  — reads every frame from non_dynamic.bag, augments it, and
                 writes results to dynamic_sim.bag.
  2. Live mode — subscribes to /velodyne_points for real-time augmentation
                 while keyboard keys toggle additional simulated objects.
"""

import math
import signal
import threading
import datetime

import numpy as np
import rospy
import rosbag
import ros_numpy
from scipy import linalg as LA
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from pynput.keyboard import Key, Listener

from dynamic_points_creation import Vehicle

# ---------------------------------------------------------------------------
# Simulated vehicle objects (double-deck bus: 2.5 m wide, 12.8 m long, 3.0 m tall)
# ---------------------------------------------------------------------------
dynamic_bus_right = Vehicle(2.5, 12.8, 3.0, [4.0, -4.0, 0.0])
dynamic_bus_left  = Vehicle(2.5, 12.8, 3.0, [-4.0, -6.0, 0.0])

# ---------------------------------------------------------------------------
# Speed configuration  (m/s, assuming the input bag runs at ~10 Hz)
#
# How it maps to move():
#   delta (m/frame) = speed_m_s / 10
#   e.g. 20 m/s → 2.0 m per frame → 20 m/s × 3.6 ≈ 72 km/h
#
# Typical values:
#    5 m/s ≈  18 km/h  slow / traffic jam
#   10 m/s ≈  36 km/h  urban street
#   20 m/s ≈  72 km/h  arterial road   ← default
#   30 m/s ≈ 108 km/h  highway
#
# Set negative to make a bus travel in the opposite direction.
# ---------------------------------------------------------------------------
BUS_RIGHT_SPEED_M_S = 5.0
BUS_LEFT_SPEED_M_S  = 5.0

# ---------------------------------------------------------------------------
# Runtime state
# ---------------------------------------------------------------------------
time_lidarstamp = rospy.Time()
ros_bag = None   # opened in __main__ after init_node
pub_sim = None   # publisher created in __main__ after init_node

# ---------------------------------------------------------------------------
# Fast PointCloud2 serialisation — packed struct, no Python loops
# ---------------------------------------------------------------------------
# Field layout (22 bytes/point): x(f4) y(f4) z(f4) intensity(f4) ring(u2) time(f4)
# Explicit offsets prevent numpy from inserting alignment padding.
_POINT_DTYPE = np.dtype({
    'names':    ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats':  ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets':  [0, 4, 8, 12, 16, 18],
    'itemsize': 22,
})

_POINT_FIELDS = [
    PointField('x',         0,  PointField.FLOAT32, 1),
    PointField('y',         4,  PointField.FLOAT32, 1),
    PointField('z',         8,  PointField.FLOAT32, 1),
    PointField('intensity', 12, PointField.FLOAT32, 1),
    PointField('ring',      16, PointField.UINT16,  1),
    PointField('time',      18, PointField.FLOAT32, 1),
]


def _to_pointcloud2(points, stamp, frame_id='velodyne'):
    """Serialise an (N,6) float32 array to PointCloud2 via direct binary packing.

    Avoids any Python-level per-point iteration; the only overhead is six
    numpy slice assignments and one tobytes() call.
    """
    n = len(points)
    buf = np.empty(n, dtype=_POINT_DTYPE)
    buf['x']         = points[:, 0]
    buf['y']         = points[:, 1]
    buf['z']         = points[:, 2]
    buf['intensity']  = points[:, 3]
    buf['ring']       = points[:, 4].astype(np.uint16)
    buf['time']       = points[:, 5]

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp    = stamp
    msg.height      = 1
    msg.width       = n
    msg.fields      = _POINT_FIELDS
    msg.is_bigendian = False
    msg.point_step  = 22
    msg.row_step    = 22 * n
    msg.data        = buf.tobytes()
    msg.is_dense    = True
    return msg


# ---------------------------------------------------------------------------
# Point-cloud publishing
# ---------------------------------------------------------------------------

def pub_ros(ros_pointcloud):
    """Publish merged cloud on /points_simulated and record it to the output bag."""
    global time_lidarstamp, ros_bag, pub_sim

    points = ros_pointcloud[:, :6]
    num_dynamic = int((points[:, 5] == 10).sum())
    num_pc      = len(points)
    num_static  = num_pc - num_dynamic

    log_line = (
        f"{time_lidarstamp.to_sec():.6f} "
        f"num_dynamic {num_dynamic} num_static {num_static} "
        f"num_pointcloud {num_pc} "
        f"percentage {num_dynamic / num_pc:.4f}"
    )
    print(log_line)

    with open('dynamic_stat.txt', 'a') as f:
        f.write(log_line + "\n")

    msg = _to_pointcloud2(points, time_lidarstamp)
    ros_bag.write('/simulated_pointcloud', msg, time_lidarstamp)
    pub_sim.publish(msg)


# ---------------------------------------------------------------------------
# Occlusion culling + vehicle injection  (fully vectorised)
# ---------------------------------------------------------------------------

def augment_pointcloud(pcd_np):
    """Remove occluded background points and merge in simulated vehicle clouds.

    All per-point filtering is done with numpy boolean masks — no Python loops.

    For each simulated bus, points whose azimuth falls inside the vehicle's
    projected angular extent and that satisfy the lateral/depth occlusion
    conditions are removed.  The vehicle's own points are then concatenated.

    Args:
        pcd_np: (N, 6) float32 array — the current merged point cloud.
    """
    right_edges = dynamic_bus_right.get_current_edge()
    left_edges  = dynamic_bus_left.get_current_edge()

    # Pre-compute sorted angular extents once (8 corners each)
    right_azims = sorted(math.atan2(e[0], e[1]) for e in right_edges)
    right_elevs = sorted(
        math.degrees(math.atan2(e[2], math.sqrt(e[0]**2 + e[1]**2)))
        for e in right_edges
    )
    left_azims = sorted(math.atan2(e[0], e[1]) for e in left_edges)
    left_elevs = sorted(
        math.degrees(math.atan2(e[2], math.sqrt(e[0]**2 + e[1]**2)))
        for e in left_edges
    )

    # --- Vectorised per-point geometry (replaces the Python for-loop) ---
    x, y, z = pcd_np[:, 0], pcd_np[:, 1], pcd_np[:, 2]
    r_xy  = np.sqrt(x**2 + y**2)
    azims = np.arctan2(x, y)
    elevs = np.degrees(np.arctan2(z, r_xy))

    # Right bus — angular zone mask
    in_right = (
        (azims > right_azims[0]) & (azims < right_azims[7]) &
        (elevs < right_elevs[7])
    )
    r_straddles = right_edges[0][1] * right_edges[4][1] < 0
    cull_right = in_right & (
        ((x > right_edges[0][0]) & r_straddles) |
        ((y > 0) & (x > right_edges[0][0]) & (y > right_edges[5][1])) |
        ((y < 0) & (x > right_edges[0][0]) & (y < right_edges[0][1]))
    )

    # Left bus — angular zone mask
    in_left = (
        (azims > left_azims[0]) & (azims < left_azims[7]) &
        (elevs < left_elevs[7])
    )
    l_straddles = left_edges[0][1] * left_edges[4][1] < 0
    cull_left = in_left & (
        ((x < left_edges[1][0]) & l_straddles) |
        ((y > 0) & (x < left_edges[5][0]) & (y > left_edges[5][1])) |
        ((y < 0) & (x < left_edges[1][0]) & (y < left_edges[1][1]))
    )

    filtered = pcd_np[~(cull_right | cull_left)]
    merged = np.concatenate(
        (filtered, dynamic_bus_right.get_current_pc(), dynamic_bus_left.get_current_pc()),
        axis=0,
    )

    pub_ros(merged)


# ---------------------------------------------------------------------------
# Main point-cloud callback
# ---------------------------------------------------------------------------

def pointcloud_callback(raw_pointcloud):
    """Process one LiDAR frame: advance vehicles, compute GDOP, publish."""
    global time_lidarstamp

    time_lidarstamp = raw_pointcloud.header.stamp
    print(f"Frame at {datetime.datetime.now()} — stamp {time_lidarstamp.to_sec():.3f}")

    # Fast structured-array → regular array conversion (no Python tolist())
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(raw_pointcloud)
    pcd_np = np.column_stack([
        arr['x'].astype(np.float32),
        arr['y'].astype(np.float32),
        arr['z'].astype(np.float32),
        arr['intensity'].astype(np.float32),
        arr['ring'].astype(np.float32),
        arr['time'].astype(np.float32),
    ])

    # Advance both buses along the Y axis
    pcd_np = np.concatenate((pcd_np, dynamic_bus_right.move(BUS_RIGHT_SPEED_M_S)), axis=0)
    pcd_np = np.concatenate((pcd_np, dynamic_bus_left.move(BUS_LEFT_SPEED_M_S)), axis=0)

    # Geometric dilution of precision over dynamic points
    dynamic_pts = pcd_np[pcd_np[:, 5] == 10, 0:3]
    if len(dynamic_pts) > 3:
        coords = dynamic_pts - np.mean(dynamic_pts, axis=0)
        cov = np.cov(coords, rowvar=False)
        evals, evecs = LA.eigh(cov)
        norms = np.linalg.norm(dynamic_pts, axis=1, keepdims=True)
        norms = np.where(norms == 0, 1e-9, norms)
        unit_vecs = dynamic_pts / norms
        H = np.linalg.inv(unit_vecs.T @ unit_vecs)
        gdop = math.sqrt(H[0, 0] + H[1, 1] + H[2, 2])
        print(f"GDOP: {gdop:.4f}  dynamic pts: {len(dynamic_pts)}")

    augment_pointcloud(pcd_np)


# ---------------------------------------------------------------------------
# Keyboard controls (live mode)
# ---------------------------------------------------------------------------
# Keys toggle boolean flags that can gate additional simulated objects.
# Extend pointcloud_callback to use these flags for custom scenarios.
sim_front        = False
sim_left         = False
sim_right        = False
sim_rear         = False
sim_right_top    = False
sim_right_bottom = False


def on_press(key):
    global sim_front, sim_left, sim_right, sim_rear, sim_right_top, sim_right_bottom

    k = str(key)
    if k == "'w'":
        sim_front = not sim_front;         print(f"sim_front        = {sim_front}")
    elif k == "'a'":
        sim_left = not sim_left;           print(f"sim_left         = {sim_left}")
    elif k == "'d'":
        sim_right = not sim_right;         print(f"sim_right        = {sim_right}")
    elif k == "'s'":
        sim_rear = not sim_rear;           print(f"sim_rear         = {sim_rear}")
    elif k == "'e'":
        sim_right_top = not sim_right_top; print(f"sim_right_top    = {sim_right_top}")
    elif k == "'c'":
        sim_right_bottom = not sim_right_bottom; print(f"sim_right_bottom = {sim_right_bottom}")


def on_release(key):
    pass


def _keyboard_listener_thread(_):
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()


# ---------------------------------------------------------------------------
# Signal handler
# ---------------------------------------------------------------------------

def shutdown_handler(signum, frame):
    global ros_bag
    print("\nShutting down — closing output bag.")
    if ros_bag:
        ros_bag.close()
    raise SystemExit(0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node('lidar_vehicle_sim', anonymous=True)

    pub_sim = rospy.Publisher('/points_simulated', PointCloud2, queue_size=10)
    ros_bag = rosbag.Bag('dynamic_sim.bag', 'w')

    # Reset statistics log
    open('dynamic_stat.txt', 'w').close()

    signal.signal(signal.SIGINT,  shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    # Subscribe to live Velodyne data for interactive mode
    rospy.Subscriber('/velodyne_points', PointCloud2, pointcloud_callback)

    # Process the static scene bag first
    print("Processing non_dynamic.bag ...")
    input_bag = rosbag.Bag('non_dynamic.bag')
    for topic, msg, t in input_bag.read_messages(topics=['/simulated_pointcloud']):
        pointcloud_callback(msg)
    input_bag.close()
    print("Done. Output written to dynamic_sim.bag")

    # Enter live keyboard-controlled mode
    t1 = threading.Thread(target=_keyboard_listener_thread, args=('',))
    t1.start()
    t1.join()

    rospy.spin()
