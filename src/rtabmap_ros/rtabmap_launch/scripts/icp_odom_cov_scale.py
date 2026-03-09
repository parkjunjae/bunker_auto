#!/usr/bin/env python3
"""
ICP odometry vyaw covariance dynamic scaler.

[정지]  vyaw=0 강제 + cov=0.001 → EKF가 vyaw=0을 신뢰 → 드리프트 즉시 중단
[회전]  ICP vyaw 그대로 + cov*1 → EKF가 ICP 완전 추적
[전진]  ICP vyaw 그대로 + cov*10 → EKF가 ICP 부분 신뢰
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

SCALE_ROT   =    1.0   # 회전: ICP 완전 신뢰
SCALE_TRANS =   10.0   # 전진: ICP 부분 신뢰
STOP_COV    =    0.001  # 정지: vyaw=0의 covariance (작을수록 EKF가 빨리 0으로 수렴)

LIN_THRESH = 0.01   # m/s
ANG_THRESH = 0.01   # rad/s
CMD_TIMEOUT = 0.5

MODE_STOP  = 0
MODE_ROT   = 1
MODE_TRANS = 2
MODE_NAMES = {
    MODE_STOP: 'STOP',
    MODE_ROT: 'ROT',
    MODE_TRANS: 'TRANS',
}


class IcpOdomCovScale(Node):
    def __init__(self):
        super().__init__('icp_odom_cov_scale')
        self._mode = MODE_STOP
        self._last_cmd_time = self.get_clock().now()
        self._last_cmd_lin = 0.0
        self._last_cmd_ang = 0.0
        self._last_log_time = self.get_clock().now()

        self.pub = self.create_publisher(Odometry, '/icp_odom_filtered', 10)
        self.create_subscription(Odometry, '/icp_odom', self._icp_cb, qos_profile_sensor_data)
        self.create_subscription(Twist,    '/cmd_vel',  self._cmd_cb, 10)
        self.create_timer(0.1, self._timeout_check)

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()
        self._last_cmd_lin = msg.linear.x
        self._last_cmd_ang = msg.angular.z
        lin = abs(msg.linear.x)
        ang = abs(msg.angular.z)

        if lin < LIN_THRESH and ang < ANG_THRESH:
            self._mode = MODE_STOP
        elif ang >= ANG_THRESH:
            self._mode = MODE_ROT
        else:
            self._mode = MODE_TRANS

    def _timeout_check(self):
        dt = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if dt > CMD_TIMEOUT:
            self._mode = MODE_STOP

    def _icp_cb(self, msg: Odometry):
        cov = list(msg.twist.covariance)
        raw_vyaw = msg.twist.twist.angular.z
        raw_cov35 = cov[35]

        if self._mode == MODE_STOP:
            # ★ 핵심: vyaw 값을 0으로 강제 + 낮은 covariance
            # EKF가 "vyaw=0"을 확실히 신뢰 → 내부 vyaw 상태가 즉시 0으로 수렴
            # → yaw 적분 중단 → 드리프트 없음
            msg.twist.twist.angular.z = 0.0
            cov[35] = STOP_COV
        elif self._mode == MODE_ROT:
            cov[35] = cov[35] * SCALE_ROT
        else:
            cov[35] = cov[35] * SCALE_TRANS

        out_vyaw = msg.twist.twist.angular.z
        out_cov35 = cov[35]
        now = self.get_clock().now()
        cmd_age = (now - self._last_cmd_time).nanoseconds * 1e-9
        if (now - self._last_log_time).nanoseconds >= 1_000_000_000:
            self._last_log_time = now
            self.get_logger().info(
                f'mode={MODE_NAMES.get(self._mode, "UNKNOWN")} '
                f'raw_vyaw={raw_vyaw:.6f} raw_cov35={raw_cov35:.6f} '
                f'out_vyaw={out_vyaw:.6f} out_cov35={out_cov35:.6f} '
                f'cmd_lin={self._last_cmd_lin:.6f} cmd_ang={self._last_cmd_ang:.6f} '
                f'cmd_age={cmd_age:.3f}s')

        msg.twist.covariance = cov
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = IcpOdomCovScale()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
