#!/usr/bin/env python3
"""
map_tf_stabilizer.py

제자리 회전 중 RTAB-Map의 map→odom TF가 흔들리는 문제를 해결.
순수 회전(linear≈0, angular>thresh) 구간에서 회전 직전 map→odom을
고주파로 재발행해 RTAB-Map의 느린(5Hz) 업데이트를 덮어씀.

회전 종료 후에는 RTAB-Map이 정착할 시간을 주고,
그 사이 locked TF를 계속 발행해 갑작스러운 점프를 방지.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import tf2_ros

ROTATE_ANG_THRESH = 0.08   # rad/s: 이 이상이면 회전으로 판정
ROTATE_LIN_THRESH = 0.05   # m/s:  이 이하여야 순수 회전으로 판정
SETTLE_SEC        = 3.0    # 회전 종료 후 locked TF를 유지할 시간(s)
PUBLISH_HZ        = 20.0   # locked TF 발행 주파수 (RTAB-Map 5Hz 덮어쓰기)


class MapTfStabilizer(Node):
    def __init__(self):
        super().__init__('map_tf_stabilizer')

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._locked_tf      = None   # 고정할 map→odom 변환
        self._rotating       = False
        self._settle_until   = None   # 회전 종료 후 유지 기한

        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)

    # ------------------------------------------------------------------ #
    def _cmd_cb(self, msg: Twist):
        lin = abs(msg.linear.x)
        ang = abs(msg.angular.z)
        now = self.get_clock().now()

        is_pure_rotation = (ang >= ROTATE_ANG_THRESH and lin < ROTATE_LIN_THRESH)

        if is_pure_rotation:
            if not self._rotating:
                # 회전 시작: 현재 map→odom을 잠근다
                tf = self._lookup_map_odom()
                if tf is not None:
                    self._locked_tf  = tf
                    self._rotating   = True
                    self._settle_until = None
                    self.get_logger().info(
                        '[MapTfStabilizer] rotation started → map→odom locked')
        else:
            if self._rotating:
                # 회전 종료: settle 구간 시작
                self._rotating   = False
                self._settle_until = now + Duration(seconds=SETTLE_SEC)
                self.get_logger().info(
                    f'[MapTfStabilizer] rotation ended → hold locked TF for {SETTLE_SEC}s')

    # ------------------------------------------------------------------ #
    def _publish_cb(self):
        now = self.get_clock().now()

        if self._rotating:
            # 순수 회전 구간: locked TF 고주파 발행
            self._republish(now)

        elif self._settle_until is not None:
            if now < self._settle_until:
                # settle 구간: locked TF 유지 (RTAB-Map이 정착할 때까지)
                self._republish(now)
            else:
                # settle 완료: 해제
                self._locked_tf    = None
                self._settle_until = None
                self.get_logger().info(
                    '[MapTfStabilizer] settle complete → RTAB-Map TF resumed')

    # ------------------------------------------------------------------ #
    def _republish(self, now):
        if self._locked_tf is None:
            return
        t = self._locked_tf
        t.header.stamp = now.to_msg()
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------ #
    def _lookup_map_odom(self):
        try:
            return self.tf_buffer.lookup_transform(
                'map', 'odom',
                rclpy.time.Time(),           # 가장 최근 TF
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'[MapTfStabilizer] lookup failed: {e}')
            return None


def main():
    rclpy.init()
    node = MapTfStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()