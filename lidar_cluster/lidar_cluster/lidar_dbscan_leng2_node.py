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


def diameter_and_endpoints(points_xy: np.ndarray, max_points: int = 250) -> Tuple[float, np.ndarray, np.ndarray]:
    """
    클러스터 직경(diameter) = 클러스터 내부에서 가장 멀리 떨어진 두 점 사이 거리
    그리고 그 두 끝점(endpoints) 반환.
    - 점이 너무 많으면 max_points 만큼 샘플링해서 근사
    """
    n = points_xy.shape[0]
    if n < 2:
        p = points_xy[0] if n == 1 else np.array([0.0, 0.0], dtype=np.float32)
        return 0.0, p, p

    if n > max_points:
        idx = np.random.choice(n, size=max_points, replace=False)
        P = points_xy[idx]
    else:
        P = points_xy

    # pairwise distances (O(m^2), m<=max_points)
    diff = P[:, None, :] - P[None, :, :]
    dist2 = np.sum(diff * diff, axis=2)

    i, j = np.unravel_index(np.argmax(dist2), dist2.shape)
    p1 = P[i]
    p2 = P[j]
    dia = float(math.sqrt(float(dist2[i, j])))
    return dia, p1, p2


class LidarDBSCANNode(Node):
    """
    DBSCAN + centroid + L + rmin/rmax + diameter 표시
    + 직경 endpoints 점 표시 + 직경 선 표시
    """

    def __init__(self):
        super().__init__('lidar_dbscan_node')
        self.prev_p1 = None
        self.prev_p2 = None
        self.alpha = 0.8

        # ===== Parameters =====
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('frame_id', '')  # empty: use scan.header.frame_id

        # DBSCAN
        self.declare_parameter('eps', 0.25)
        self.declare_parameter('min_samples', 8)  #6->8

        # cluster filtering
        self.declare_parameter('min_cluster_size', 6)
        self.declare_parameter('max_cluster_size', 2000)

        # display sizes
        self.declare_parameter('centroid_sphere_radius', 0.08)
        self.declare_parameter('text_size', 0.08)
        self.declare_parameter('text_z', 0.30)

        # diameter calc
        self.declare_parameter('diameter_max_points', 250)  # 많으면 샘플링 근사
        self.declare_parameter('show_diameter_line', True)
        self.declare_parameter('endpoint_sphere_radius', 0.06)
        self.declare_parameter('diameter_line_width', 0.03)

        scan_topic = self.get_parameter('scan_topic').value
        self.sub = self.create_subscription(LaserScan, scan_topic, self.cb_scan, 10)
        self.pub = self.create_publisher(MarkerArray, '/lidar_dbscan_markers', 10)

        self.get_logger().info(f"Subscribed to: {scan_topic}")
        self.get_logger().info("Publishing: /lidar_dbscan_markers")

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

        X = np.array(pts, dtype=np.float32)  # (N,2)

        # DBSCAN
        eps = float(self.get_parameter('eps').value)
        min_samples = int(self.get_parameter('min_samples').value)
        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(X)  # -1 noise

        min_sz = int(self.get_parameter('min_cluster_size').value)
        max_sz = int(self.get_parameter('max_cluster_size').value)

        clusters: List[np.ndarray] = []
        for lb in sorted(set(labels)):
            if lb == -1:
                continue
            idx = np.where(labels == lb)[0]
            if min_sz <= len(idx) <= max_sz:
                clusters.append(X[idx])

        # publish markers
        frame_id_param = self.get_parameter('frame_id').value
        frame_id = frame_id_param if frame_id_param else scan.header.frame_id
        now = self.get_clock().now().to_msg()

        ma = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        mid = 0

        # --- LiDAR 위치 표시 (노란 점) ---
        lidar_marker = Marker()
        lidar_marker.header.frame_id = frame_id
        lidar_marker.header.stamp = now
        lidar_marker.ns = "lidar_position"
        lidar_marker.id = mid
        mid += 1

        lidar_marker.type = Marker.SPHERE
        lidar_marker.action = Marker.ADD

        lidar_marker.pose.position.x = 0.0
        lidar_marker.pose.position.y = 0.0
        lidar_marker.pose.position.z = 0.0

        lidar_marker.scale.x = 0.15
        lidar_marker.scale.y = 0.15
        lidar_marker.scale.z = 0.15

        # 노란색
        lidar_marker.color.r = 1.0
        lidar_marker.color.g = 1.0
        lidar_marker.color.b = 0.0
        lidar_marker.color.a = 1.0

        ma.markers.append(lidar_marker)

        sphere_radius = float(self.get_parameter('centroid_sphere_radius').value)
        text_size = float(self.get_parameter('text_size').value)
        text_z = float(self.get_parameter('text_z').value)

        dia_max_pts = int(self.get_parameter('diameter_max_points').value)
        show_line = bool(self.get_parameter('show_diameter_line').value)
        endpoint_r = float(self.get_parameter('endpoint_sphere_radius').value)
        line_w = float(self.get_parameter('diameter_line_width').value)

        for k, c in enumerate(clusters):
            # centroid
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))

            # L (AABB 긴 변)
            xmin = float(np.min(c[:, 0])); xmax = float(np.max(c[:, 0]))
            ymin = float(np.min(c[:, 1])); ymax = float(np.max(c[:, 1]))
            width = xmax - xmin
            height = ymax - ymin
            L = max(width, height)

            # rmin / rmax (원점 기준)
            ranges = np.sqrt(c[:, 0] ** 2 + c[:, 1] ** 2)
            rmin = float(np.min(ranges))
            rmax = float(np.max(ranges))

            # diameter + endpoints
            dia, p1, p2 = diameter_and_endpoints(c, max_points=dia_max_pts)

            p1_range = math.sqrt(p1[0]**2 + p1[1]**2)
            p2_range = math.sqrt(p2[0]**2 + p2[1]**2)

            # temporal filtering
            if self.prev_p1 is None:
                self.prev_p1 = p1_range
                self.prev_p2 = p2_range
            else:
                p1_range = self.alpha * self.prev_p1 + (1-self.alpha) * p1_range
                p2_range = self.alpha * self.prev_p2 + (1-self.alpha) * p2_range

                self.prev_p1 = p1_range
                self.prev_p2 = p2_range

            # centroid sphere
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = now
            m.ns = "centroids"
            m.id = mid; mid += 1
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

            # text
            t = Marker()
            t.header.frame_id = frame_id
            t.header.stamp = now
            t.ns = "cluster_info"
            t.id = mid; mid += 1
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
            t.text = f"rmin={rmin:.2f}m\nrmax={rmax:.2f}m\ndia={dia:.2f}m\np1={p1_range:.2f}m\np2={p2_range:.2f}m"
            ma.markers.append(t)

            # endpoints spheres
            e1 = Marker()
            e1.header.frame_id = frame_id
            e1.header.stamp = now
            e1.ns = "dia_endpoints"
            e1.id = mid; mid += 1
            e1.type = Marker.SPHERE
            e1.action = Marker.ADD
            e1.pose.position.x = float(p1[0])
            e1.pose.position.y = float(p1[1])
            e1.pose.position.z = 0.0
            e1.scale.x = endpoint_r
            e1.scale.y = endpoint_r
            e1.scale.z = endpoint_r
            e1.color.a = 1.0
            e1.color.r = 0.2
            e1.color.g = 1.0
            e1.color.b = 0.2
            ma.markers.append(e1)

            e2 = Marker()
            e2.header.frame_id = frame_id
            e2.header.stamp = now
            e2.ns = "dia_endpoints"
            e2.id = mid; mid += 1
            e2.type = Marker.SPHERE
            e2.action = Marker.ADD
            e2.pose.position.x = float(p2[0])
            e2.pose.position.y = float(p2[1])
            e2.pose.position.z = 0.0
            e2.scale.x = endpoint_r
            e2.scale.y = endpoint_r
            e2.scale.z = endpoint_r
            e2.color.a = 1.0
            e2.color.r = 0.2
            e2.color.g = 1.0
            e2.color.b = 0.2
            ma.markers.append(e2)

            # diameter line
            if show_line:
                line = Marker()
                line.header.frame_id = frame_id
                line.header.stamp = now
                line.ns = "cluster_shape"
                line.id = mid
                mid += 1

                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD

                line.scale.x = 0.03

                line.color.r = 0.0
                line.color.g = 1.0
                line.color.b = 1.0
                line.color.a = 1.0

                line.points = []

                for pt in c:
                    p = Point()
                    p.x = float(pt[0])
                    p.y = float(pt[1])
                    p.z = 0.0
                    line.points.append(p)

                ma.markers.append(line)

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