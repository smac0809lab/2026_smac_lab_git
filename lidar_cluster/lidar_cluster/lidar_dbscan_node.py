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
    LaserScan -> (x,y) points -> DBSCAN -> clusters + centroids -> RViz MarkerArray
    """

    def __init__(self):
        super().__init__('lidar_dbscan_node')

        # ===== Parameters =====
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('frame_id', '')  # empty: use scan.header.frame_id

        self.declare_parameter('range_min_override', 0.0)  # 0이면 scan.range_min 사용
        self.declare_parameter('range_max_override', 0.0)  # 0이면 scan.range_max 사용

        # DBSCAN params
        self.declare_parameter('eps', 0.25)            # meters (핵심 튜닝값)
        self.declare_parameter('min_samples', 6)       # 클러스터 최소 점 개수

        # Post-filtering (노이즈/벽 처리용)
        self.declare_parameter('min_cluster_size', 6)
        self.declare_parameter('max_cluster_size', 2000)

        # Visualization
        self.declare_parameter('publish_cluster_points', True)
        self.declare_parameter('centroid_sphere_radius', 0.08)

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(LaserScan, scan_topic, self.cb_scan, 10)

        self.pub = self.create_publisher(MarkerArray, '/lidar_dbscan_markers', 10)

        self.get_logger().info(f"Subscribed to: {scan_topic}")
        self.get_logger().info("Publishing MarkerArray: /lidar_dbscan_markers")

    def cb_scan(self, scan: LaserScan):
        # ----- range limits -----
        rmin = scan.range_min
        rmax = scan.range_max
        rmin_over = float(self.get_parameter('range_min_override').value)
        rmax_over = float(self.get_parameter('range_max_override').value)
        if rmin_over > 0.0:
            rmin = rmin_over
        if rmax_over > 0.0:
            rmax = rmax_over

        # ----- LaserScan -> XY points -----
        pts: List[Tuple[float, float]] = []
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and (rmin <= r <= rmax):
                pts.append(polar_to_xy(r, angle))
            angle += scan.angle_increment

        if len(pts) < 10:
            return

        X = np.array(pts, dtype=np.float32)  # shape (N,2)

        # ----- DBSCAN -----
        eps = float(self.get_parameter('eps').value)
        min_samples = int(self.get_parameter('min_samples').value)
        db = DBSCAN(eps=eps, min_samples=min_samples)
        labels = db.fit_predict(X)  # -1 = noise

        # ----- Gather clusters -----
        min_sz = int(self.get_parameter('min_cluster_size').value)
        max_sz = int(self.get_parameter('max_cluster_size').value)

        clusters = []
        unique_labels = sorted(set(labels))
        for lb in unique_labels:
            if lb == -1:
                continue  # noise skip
            idx = np.where(labels == lb)[0]
            if len(idx) < min_sz or len(idx) > max_sz:
                continue
            clusters.append(X[idx])

        # ----- Centroids -----
        centroids = []
        for c in clusters:
            centroids.append((float(np.mean(c[:, 0])), float(np.mean(c[:, 1]))))

        # ----- Publish Markers -----
        frame_id_param = self.get_parameter('frame_id').value
        frame_id = frame_id_param if frame_id_param else scan.header.frame_id
        now = self.get_clock().now().to_msg()

        ma = MarkerArray()

        # delete all old markers
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        publish_cluster_points = bool(self.get_parameter('publish_cluster_points').value)
        sphere_radius = float(self.get_parameter('centroid_sphere_radius').value)

        mid = 0

        # (optional) cluster points
        if publish_cluster_points:
            for k, c in enumerate(clusters):
                m = Marker()
                m.header.frame_id = frame_id
                m.header.stamp = now
                m.ns = "dbscan_cluster_points"
                m.id = mid
                mid += 1
                m.type = Marker.POINTS
                m.action = Marker.ADD
                m.scale.x = 0.03
                m.scale.y = 0.03

                # 보기 좋게만: 클러스터마다 색 조금씩 바꾸기
                # (원하면 이 부분 지워도 됨)
                m.color.a = 0.9
                m.color.r = float((k * 53 % 255) / 255.0)
                m.color.g = float((k * 97 % 255) / 255.0)
                m.color.b = float((k * 193 % 255) / 255.0)

                m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in c]
                ma.markers.append(m)

        # centroid spheres
        for (cx, cy) in centroids:
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = now
            m.ns = "dbscan_centroids"
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

        self.pub.publish(ma)


def main():
    rclpy.init()
    node = LidarDBSCANNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
