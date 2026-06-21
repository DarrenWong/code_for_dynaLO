#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import numpy as np
from scipy.spatial.transform import Rotation as R


class Vehicle:
    """Hollow bounding-box model of a vehicle as a LiDAR point cloud.

    Points are placed on the six faces of an axis-aligned box at a fixed
    spatial resolution (10 pts/m).  Column layout of points_array:
        [x, y, z, intensity, ring, label]
    where label 10 = dynamic, 5 = static.
    """

    def __init__(self, x, y, z, position, is_static=False):
        """
        Args:
            x, y, z:    Vehicle dimensions in metres (width, length, height).
            position:   [x, y, z] world-frame centre.
            is_static:  Label points as static (5) if True, dynamic (10) if False.
        """
        self.x = x
        self.y = y
        self.z = z
        self.position = list(position)
        self.rotation = [0, 0, 0, 1]

        res = 10.0          # points per metre
        x_half = x / 2
        y_half = y / 2
        z_half = z / 2
        xi = int(x * res / 2)
        yi = int(y * res / 2)
        zi = int(z * res / 2)

        i_vals = np.arange(-xi, xi, dtype=np.float32) / res   # x grid
        j_vals = np.arange(-yi, yi, dtype=np.float32) / res   # y grid
        k_vals = np.arange(-zi, zi, dtype=np.float32) / res   # z grid

        def _face(col_a, a_vals, col_b, b_vals, col_c, const_val):
            """Build one face as (N,6) array using meshgrid — no Python loops."""
            aa, bb = np.meshgrid(a_vals, b_vals, indexing='ij')
            aa, bb = aa.ravel(), bb.ravel()
            n = len(aa)
            pts = np.empty((n, 6), dtype=np.float32)
            pts[:, col_a] = aa
            pts[:, col_b] = bb
            pts[:, col_c] = const_val
            pts[:, 3] = 60.0
            pts[:, 4] = 1.0
            pts[:, 5] = 1.0
            return pts

        # Left / right faces  — vary y (j) and z (k), fix x
        # Top  / bottom faces — vary x (i) and y (j), fix z
        # Front / rear faces  — vary x (i) and z (k), fix y
        self.points_array = np.vstack([
            _face(1, j_vals, 2, k_vals, 0, -x_half),   # left
            _face(1, j_vals, 2, k_vals, 0,  x_half),   # right
            _face(0, i_vals, 1, j_vals, 2, -z_half),   # bottom
            _face(0, i_vals, 1, j_vals, 2,  z_half),   # top
            _face(0, i_vals, 2, k_vals, 1, -y_half),   # rear
            _face(0, i_vals, 2, k_vals, 1,  y_half),   # front
        ])

        self.points_array[:, 0] += position[0]
        self.points_array[:, 1] += position[1]
        self.points_array[:, 2] += position[2]

        self.points_array[:, 5] = 5 if is_static else 10
        print(f"Creating vehicle: size=({x}, {y}, {z}), position={position}, "
              f"points={len(self.points_array)}")

    def move(self, speed):
        """Advance the vehicle along the Y axis by one LiDAR frame.

        The displacement per call is speed / 10 metres.  At a typical bag
        rate of 10 Hz this makes `speed` numerically equal to m/s:
            move(10)  →  1.0 m/frame × 10 Hz  =  10 m/s  ≈  36 km/h
            move(20)  →  2.0 m/frame × 10 Hz  =  20 m/s  ≈  72 km/h

        Pass a negative value to reverse direction.

        Wraps around at Y = +20 m with a -60 m reset so the vehicle loops
        continuously through the sensor field of view.

        Args:
            speed: Desired speed in m/s (for a 10 Hz input bag).

        Returns:
            Updated points_array (N x 6).
        """
        delta = speed / 10
        self.points_array[:, 1] += delta
        self.position[1] += delta
        if self.position[1] > 20:
            self.points_array[:, 1] -= 60
            self.position[1] -= 60
        return self.points_array

    def transform_to_local(self, ego_pose, static_object_pose, static_object_rotation=None):
        """Transform vehicle points from world frame to the ego vehicle's local frame.

        Args:
            ego_pose:               nav_msgs/Odometry containing ego pose in world frame.
            static_object_pose:     [x, y, z] world-frame position of this vehicle.
            static_object_rotation: [x, y, z, w] quaternion; identity if None.

        Returns:
            Points in local frame (N x 6).
        """
        if static_object_rotation is None:
            static_object_rotation = [0, 0, 0, 1]

        T_ego = np.eye(4)
        r_ego = R.from_quat([
            ego_pose.pose.pose.orientation.x,
            ego_pose.pose.pose.orientation.y,
            ego_pose.pose.pose.orientation.z,
            ego_pose.pose.pose.orientation.w,
        ])
        T_ego[0:3, 0:3] = r_ego.as_matrix()
        T_ego[0:3, 3] = [
            ego_pose.pose.pose.position.x,
            ego_pose.pose.pose.position.y,
            ego_pose.pose.pose.position.z,
        ]

        T_obj = np.eye(4)
        T_obj[0:3, 0:3] = R.from_quat(static_object_rotation).as_matrix()
        T_obj[0:3, 3] = np.array(static_object_pose)

        T_local = np.linalg.inv(T_ego) @ T_obj

        points_local = self.points_array.copy()
        rotated = T_local[0:3, 0:3] @ self.points_array[:, 0:3].T
        points_local[:, 0:3] = rotated.T
        points_local[:, 0] += float(T_local[0, 3])
        points_local[:, 1] += float(T_local[1, 3])
        points_local[:, 2] += float(T_local[2, 3])

        self.position[0] = float(T_local[0, 3])
        self.position[1] = float(T_local[1, 3])
        self.position[2] = float(T_local[2, 3])
        self.rotation = R.from_matrix(T_local[0:3, 0:3]).as_quat().tolist()

        return points_local

    def get_current_pc(self):
        """Return the vehicle's current point cloud (N x 6)."""
        return self.points_array

    def get_size(self):
        """Return vehicle dimensions [x, y, z] in metres."""
        return [self.x, self.y, self.z]

    def get_current_edge(self):
        """Return the 8 corner points of the axis-aligned bounding box.

        Order: front_left_top, front_right_top, front_left_bottom,
               front_right_bottom, rear_left_top, rear_right_top,
               rear_left_bottom, rear_right_bottom.
        """
        px, py, pz = self.position[0], self.position[1], self.position[2]
        hx, hy, hz = self.x / 2, self.y / 2, self.z / 2
        return [
            [px - hx, py + hy, pz + hz],
            [px + hx, py + hy, pz + hz],
            [px - hx, py + hy, pz - hz],
            [px + hx, py + hy, pz - hz],
            [px - hx, py - hy, pz + hz],
            [px + hx, py - hy, pz + hz],
            [px - hx, py - hy, pz - hz],
            [px + hx, py - hy, pz - hz],
        ]

    def get_current_position(self):
        """Return current centre position [x, y, z]."""
        return self.position

    def get_current_rotation(self):
        """Return current orientation as [x, y, z, w] quaternion."""
        return self.rotation
