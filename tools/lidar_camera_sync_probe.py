#!/usr/bin/env python3
"""
LiDAR-카메라 타임스탬프 정합 점검 스크립트.

기본:
- LiDAR:  /livox/lidar/synced/deskewed (sensor_msgs/PointCloud2)
- Camera: /camera/camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)

출력:
- camera_stamp - lidar_stamp 통계(평균/중앙값/표준편차)
- 현재 lidar_offset_sec 기준 권장 보정값
"""

import argparse
import bisect
import statistics
import time
from collections import deque
from typing import Deque, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SyncProbe(Node):
    def __init__(self, args):
        super().__init__("lidar_camera_sync_probe")
        self.args = args
        self.start_wall = time.time()
        self.last_report_wall = 0.0

        self.camera_stamps: Deque[float] = deque(maxlen=args.buffer_size)
        self.dts: List[float] = []  # dt = cam - lidar
        self.lidar_count = 0
        self.cam_count = 0
        self.match_count = 0

        self.create_subscription(
            Image, args.camera_topic, self.cb_camera, 100
        )
        self.create_subscription(
            PointCloud2, args.lidar_topic, self.cb_lidar, 100
        )
        self.timer = self.create_timer(0.5, self.on_timer)

        self.get_logger().info(f"camera_topic: {args.camera_topic}")
        self.get_logger().info(f"lidar_topic : {args.lidar_topic}")
        self.get_logger().info(
            "dt 정의: camera_stamp - lidar_stamp (양수면 LiDAR가 더 이른 시간)"
        )

    def cb_camera(self, msg: Image) -> None:
        self.cam_count += 1
        self.camera_stamps.append(stamp_to_sec(msg.header.stamp))

    def _nearest_camera_stamp(self, lidar_t: float) -> float:
        if not self.camera_stamps:
            return float("nan")
        arr = list(self.camera_stamps)
        i = bisect.bisect_left(arr, lidar_t)
        cand = []
        if i < len(arr):
            cand.append(arr[i])
        if i > 0:
            cand.append(arr[i - 1])
        if not cand:
            return float("nan")
        return min(cand, key=lambda t: abs(t - lidar_t))

    def cb_lidar(self, msg: PointCloud2) -> None:
        self.lidar_count += 1
        lidar_t = stamp_to_sec(msg.header.stamp)
        cam_t = self._nearest_camera_stamp(lidar_t)
        if cam_t != cam_t:  # NaN
            return
        if abs(cam_t - lidar_t) > self.args.max_match_sec:
            return

        dt = cam_t - lidar_t
        self.dts.append(dt)
        self.match_count += 1

    def _format_stats(self) -> str:
        if len(self.dts) < 5:
            return (
                f"matches={len(self.dts)} "
                f"(camera={self.cam_count}, lidar={self.lidar_count})"
            )
        mean_dt = statistics.fmean(self.dts)
        med_dt = statistics.median(self.dts)
        std_dt = statistics.pstdev(self.dts) if len(self.dts) > 1 else 0.0
        p95_abs = sorted(abs(x) for x in self.dts)[int(0.95 * (len(self.dts) - 1))]
        return (
            f"matches={len(self.dts)} "
            f"mean={mean_dt*1e3:+.2f}ms med={med_dt*1e3:+.2f}ms "
            f"std={std_dt*1e3:.2f}ms p95|dt|={p95_abs*1e3:.2f}ms "
            f"(camera={self.cam_count}, lidar={self.lidar_count})"
        )

    def on_timer(self) -> None:
        now = time.time()
        elapsed = now - self.start_wall

        if now - self.last_report_wall >= self.args.report_sec:
            self.last_report_wall = now
            self.get_logger().info(f"t={elapsed:.1f}s {self._format_stats()}")

        if self.args.duration_sec > 0.0 and elapsed >= self.args.duration_sec:
            self.finish_and_shutdown()

    def finish_and_shutdown(self) -> None:
        if len(self.dts) < 5:
            self.get_logger().warn(
                "매칭 샘플이 부족합니다. topic 이름/데이터 수신 상태를 확인하세요."
            )
            rclpy.shutdown()
            return

        mean_dt = statistics.fmean(self.dts)
        med_dt = statistics.median(self.dts)
        std_dt = statistics.pstdev(self.dts) if len(self.dts) > 1 else 0.0
        p95_abs = sorted(abs(x) for x in self.dts)[int(0.95 * (len(self.dts) - 1))]

        # dt = camera - lidar
        # lidar_stamp' = lidar_stamp + offset_sec
        # => 새 offset 권장 = 현재 offset + dt(대표값)
        rec_offset_mean = self.args.current_lidar_offset + mean_dt
        rec_offset_med = self.args.current_lidar_offset + med_dt

        self.get_logger().info("========== RESULT ==========")
        self.get_logger().info(f"samples: {len(self.dts)}")
        self.get_logger().info(
            f"dt(cam-lidar): mean={mean_dt:+.6f}s med={med_dt:+.6f}s std={std_dt:.6f}s p95|dt|={p95_abs:.6f}s"
        )
        self.get_logger().info(
            f"current lidar_offset_sec: {self.args.current_lidar_offset:+.6f}s"
        )
        self.get_logger().info(
            f"recommended lidar_offset_sec (mean): {rec_offset_mean:+.6f}s"
        )
        self.get_logger().info(
            f"recommended lidar_offset_sec (median): {rec_offset_med:+.6f}s"
        )
        self.get_logger().info(
            "적용 예: ros2 launch rtabmap_launch sensor_sync.launch.py "
            f"lidar_offset_sec:={rec_offset_med:+.6f}"
        )
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--camera-topic",
        default="/camera/camera/aligned_depth_to_color/image_raw",
        help="sensor_msgs/Image topic",
    )
    parser.add_argument(
        "--lidar-topic",
        default="/livox/lidar/synced/deskewed",
        help="sensor_msgs/PointCloud2 topic",
    )
    parser.add_argument(
        "--current-lidar-offset",
        type=float,
        default=0.003,
        help="sensor_sync.launch.py에 현재 설정된 lidar_offset_sec",
    )
    parser.add_argument(
        "--max-match-sec",
        type=float,
        default=0.20,
        help="카메라-라이다 매칭 허용 최대 시간차",
    )
    parser.add_argument(
        "--buffer-size",
        type=int,
        default=2000,
        help="카메라 타임스탬프 버퍼 크기",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=30.0,
        help="측정 시간(0 이하이면 Ctrl+C까지 계속)",
    )
    parser.add_argument(
        "--report-sec",
        type=float,
        default=2.0,
        help="중간 통계 출력 주기",
    )
    args = parser.parse_args()

    rclpy.init()
    node = SyncProbe(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish_and_shutdown()


if __name__ == "__main__":
    main()
