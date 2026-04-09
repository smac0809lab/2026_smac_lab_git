#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def polar_to_xy(r: float, theta: float) -> Tuple[float, float]:
    return (r * math.cos(theta), r * math.sin(theta))


def euclid(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class LidarClusteringNode(Node):
    """
    LaserScan -> point list -> distance-based clustering -> centroid -> RViz markers
    """

    def __init__(self):
        super().__init__('lidar_clustering_node')

        # ===== Parameters (필요하면 런치/CLI로 바꿀 수 있음) =====
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('frame_id', '')  # 비워두면 scan.header.frame_id 사용
        self.declare_parameter('range_min_override', 0.0)  # 0이면 사용 안함
        self.declare_parameter('range_max_override', 0.0)  # 0이면 사용 안함

        # 클러스터링 파라미터
        self.declare_parameter('cluster_dist_threshold', 0.20)  # m (상황따라 0.10~0.35 튜닝)
        self.declare_parameter('min_cluster_size', 4)            # 점 개수 너무 적으면 노이즈
        self.declare_parameter('max_cluster_size', 2000)         # 너무 크면(벽) 필요시 제한

        # 시각화 옵션
        self.declare_parameter('publish_cluster_points', True)   # 클러스터 포인트도 RViz에 점으로 표시
        self.declare_parameter('centroid_sphere_radius', 0.08)   # 중심점 구 크기(m)

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(LaserScan, scan_topic, self.cb_scan, 10)

        self.pub_markers = self.create_publisher(MarkerArray, '/lidar_clusters_markers', 10)

        self.get_logger().info(f"Subscribed to: {scan_topic}")
        self.get_logger().info("Publishing MarkerArray: /lidar_clusters_markers")

    def cb_scan(self, scan: LaserScan):
        # ----- range limit -----
        rmin = scan.range_min
        rmax = scan.range_max
        rmin_over = self.get_parameter('range_min_override').value
        rmax_over = self.get_parameter('range_max_override').value
        if rmin_over and rmin_over > 0.0:
            rmin = rmin_over
        if rmax_over and rmax_over > 0.0:
            rmax = rmax_over

        # ----- LaserScan -> valid XY points -----
        points: List[Tuple[float, float]] = []
        angle = scan.angle_min

        for r in scan.ranges:
            if math.isfinite(r) and (rmin <= r <= rmax):
                points.append(polar_to_xy(r, angle))
            else:
                # invalid range가 나오면 "물체 끊김"으로 간주할지 말지 선택 가능
                # 여기서는 단순히 점만 안 넣고 넘어감 (클러스터는 거리로 끊길 것)
                pass
            angle += scan.angle_increment

        if len(points) < 2:
            return

        # ----- Clustering -----
        dist_th = float(self.get_parameter('cluster_dist_threshold').value)
        min_sz = int(self.get_parameter('min_cluster_size').value)
        max_sz = int(self.get_parameter('max_cluster_size').value)

        clusters: List[List[Tuple[float, float]]] = []
        current: List[Tuple[float, float]] = [points[0]]

        for i in range(1, len(points)):
            if euclid(points[i - 1], points[i]) < dist_th:
                current.append(points[i])
            else:
                if min_sz <= len(current) <= max_sz:
                    clusters.append(current)
                current = [points[i]]

        if min_sz <= len(current) <= max_sz:
            clusters.append(current)

        # ----- Centroids -----
        centroids: List[Tuple[float, float]] = []
        for c in clusters:
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)
            centroids.append((cx, cy))

        # ----- Publish Markers -----
        frame_id_param = self.get_parameter('frame_id').value
        frame_id = frame_id_param if frame_id_param else scan.header.frame_id

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 0) Delete all old markers
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        publish_cluster_points = bool(self.get_parameter('publish_cluster_points').value)
        sphere_radius = float(self.get_parameter('centroid_sphere_radius').value)

        mid = 0

        # 1) Cluster points markers (optional)
        if publish_cluster_points:
            for idx, c in enumerate(clusters):
                m = Marker()
                m.header.frame_id = frame_id
                m.header.stamp = now
                m.ns = "cluster_points"
                m.id = mid
                mid += 1
                m.type = Marker.POINTS
                m.action = Marker.ADD
                m.scale.x = 0.03  # point size
                m.scale.y = 0.03

                # 색은 RViz에서 보이기만 하면 되니까 단순 지정 (색 지정 싫으면 RViz에서 Overwrite 가능)
                m.color.a = 0.9
                m.color.r = 0.2
                m.color.g = 0.8
                m.color.b = 1.0

                m.points = [Point(x=p[0], y=p[1], z=0.0) for p in c]
                marker_array.markers.append(m)

        # 2) Centroid sphere markers
        for idx, (cx, cy) in enumerate(centroids):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = now
            m.ns = "centroids"
            m.id = mid
            mid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = float(cx)
            m.pose.position.y = float(cy)
            m.pose.position.z = 0.0

            m.scale.x = sphere_radius
            m.scale.y = sphere_radius
            m.scale.z = sphere_radius

            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.2

            marker_array.markers.append(m)

        self.pub_markers.publish(marker_array)


def main():
    rclpy.init()
    node = LidarClusteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
