#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import csv
import os


class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')

        # 파라미터 선언
        self.declare_parameter('csv_file_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('marker_scale', 0.1)
        self.declare_parameter('line_width', 0.05)

        # 파라미터 가져오기
        self.csv_file_path = self.get_parameter('csv_file_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.marker_scale = self.get_parameter('marker_scale').get_parameter_value().double_value
        self.line_width = self.get_parameter('line_width').get_parameter_value().double_value

        # Publisher 생성
        self.marker_publisher = self.create_publisher(MarkerArray, '/trajectory_markers', 10)

        # 타이머 생성 (1Hz로 마커 퍼블리시)
        self.timer = self.create_timer(1.0, self.publish_trajectory)

        # CSV 파일에서 trajectory 데이터 로드
        self.trajectory_points = self.load_trajectory_from_csv()

        self.get_logger().info('Trajectory Visualizer 노드가 시작되었습니다.')
        self.get_logger().info(f'CSV 파일: {self.csv_file_path}')
        self.get_logger().info(f'로드된 포인트 수: {len(self.trajectory_points)}')


    def load_trajectory_from_csv(self):
        """CSV 파일에서 trajectory 포인트들을 로드합니다."""
        points = []

        if not self.csv_file_path or not os.path.exists(self.csv_file_path):
            self.get_logger().error(f'CSV 파일을 찾을 수 없습니다: {self.csv_file_path}')
            return points

        try:
            with open(self.csv_file_path, 'r') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    if len(row) >= 2:  # x, y 값이 있는지 확인
                        try:
                            # 이미 origin이 고려된 좌표 사용
                            x = float(row[0])
                            y = float(row[1])
                            velocity = float(row[2]) if len(row) > 2 else 0.0
                            points.append((x, y, velocity))
                        except ValueError:
                            continue  # 숫자로 변환할 수 없는 행은 건너뛰기

            self.get_logger().info(f'{len(points)}개의 trajectory 포인트를 로드했습니다.')

        except Exception as e:
            self.get_logger().error(f'CSV 파일 읽기 오류: {str(e)}')

        return points

    def publish_trajectory(self):
        """Trajectory를 마커로 퍼블리시합니다."""
        if not self.trajectory_points:
            return

        marker_array = MarkerArray()

        # 라인 마커 생성 (경로를 선으로 표시)
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "trajectory_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = self.line_width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8

        max_velocity = max([pt[2] for pt in self.trajectory_points]) if self.trajectory_points else 1.0

        for i, (x, y, velocity) in enumerate(self.trajectory_points):
            # 라인에 포인트 추가
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = 0.0
            line_marker.points.append(pt)

            # 개별 포인트 마커 (속도에 따른 색상)
            point_marker = Marker()
            point_marker.header.frame_id = self.frame_id
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.ns = "trajectory_points"
            point_marker.id = i + 1
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = x
            point_marker.pose.position.y = y
            point_marker.pose.position.z = 0.0
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = self.marker_scale
            point_marker.scale.y = self.marker_scale
            point_marker.scale.z = self.marker_scale

            normalized_velocity = velocity / max_velocity if max_velocity > 0 else 0.0
            point_marker.color.r = normalized_velocity
            point_marker.color.g = 0.0
            point_marker.color.b = 1.0 - normalized_velocity
            point_marker.color.a = 0.7

            marker_array.markers.append(point_marker)

        # 라인 마커 배열에 추가
        marker_array.markers.append(line_marker)

        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

