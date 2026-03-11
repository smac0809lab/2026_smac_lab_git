#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import numpy as np
from sklearn.cluster import DBSCAN


def polar_to_xy(r: float, theta: float) -> Tuple[float, float]:
    return (r * math.cos(theta), r * math.sin(theta))


class LidarDBSCANNode(Node):
    """
    DBSCAN + Centroid + Length(L=max(width,height)) only
    RViz에 L 값만 표시
    """

    def __init__(self):
        super().__init__('lidar_dbscan_node')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('frame_id', '')

        self.declare_parameter('eps', 0.25)
        self.declare_parameter('min_samples', 6)

        self.declare_parameter('min_cluster_size', 6)
        self.declare_parameter('max_cluster_size', 2000)

        self.declare_parameter('centroid_sphere_radius', 0.08)
        self.declare_parameter('text_size', 0.22)
        self.declare_parameter('text_z', 0.30)

        scan_topic = self.get_parameter('scan_topic').value
        self.sub = self.create_subscription(LaserScan, scan_topic, self.cb_scan, 10)

        self.pub = self.create_publisher(MarkerArray, '/lidar_dbscan_markers', 10)

        self.get_logger().info(f"Subscribed to: {scan_topic}")

    def cb_scan(self, scan: LaserScan):

        # LaserScan -> XY
        pts = []
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r):
                pts.append(polar_to_xy(r, angle))
            angle += scan.angle_increment

        if len(pts) < 10:
            return

        X = np.array(pts, dtype=np.float32)

        # DBSCAN
        eps = float(self.get_parameter('eps').value)
        min_samples = int(self.get_parameter('min_samples').value)
        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(X)

        min_sz = int(self.get_parameter('min_cluster_size').value)
        max_sz = int(self.get_parameter('max_cluster_size').value)

        clusters = []
        for lb in sorted(set(labels)):
            if lb == -1:
                continue
            idx = np.where(labels == lb)[0]
            if min_sz <= len(idx) <= max_sz:
                clusters.append(X[idx])

        frame_id = scan.header.frame_id
        now = self.get_clock().now().to_msg()

        ma = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        sphere_radius = float(self.get_parameter('centroid_sphere_radius').value)
        text_size = float(self.get_parameter('text_size').value)
        text_z = float(self.get_parameter('text_z').value)

        mid = 0

        for c in clusters:

            # centroid
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))

            # bounding box
            xmin = float(np.min(c[:, 0]))
            xmax = float(np.max(c[:, 0]))
            ymin = float(np.min(c[:, 1]))
            ymax = float(np.max(c[:, 1]))

            width = xmax - xmin
            height = ymax - ymin
            length = max(width, height)

            # --- Centroid Sphere ---
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = now
            m.ns = "centroids"
            m.id = mid
            mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.0
            m.scale.x = sphere_radius
            m.scale.y = sphere_radius
            m.scale.z = sphere_radius
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.2
            ma.markers.append(m)

            # --- Length Text (L만 표시) ---
            t = Marker()
            t.header.frame_id = frame_id
            t.header.stamp = now
            t.ns = "length_text"
            t.id = mid
            mid += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD

            t.pose.position.x = cx
            t.pose.position.y = cy
            t.pose.position.z = text_z

            t.scale.z = text_size
            t.color.a = 1.0
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0

            t.text = f"L={length:.2f}m"
            ma.markers.append(t)

        self.pub.publish(ma)


def main():
    rclpy.init()
    node = LidarDBSCANNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()