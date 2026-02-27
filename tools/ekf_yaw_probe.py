#!/usr/bin/env python3

import argparse
import csv
import math
import os
import statistics
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def unwrap_step(prev_raw, prev_unwrapped, raw):
    if prev_raw is None:
        return raw, raw
    delta = wrap_pi(raw - prev_raw)
    return raw, (prev_unwrapped + delta)


class EkfYawProbe(Node):
    def __init__(self, args):
        super().__init__("ekf_yaw_probe")

        self.duration = args.duration
        self.rate_hz = args.rate
        self.v_thresh = args.v_thresh
        self.w_thresh = args.w_thresh

        self.csv_path = os.path.expanduser(args.csv)
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)

        self.fp = open(self.csv_path, "w", newline="")
        self.writer = csv.DictWriter(
            self.fp,
            fieldnames=[
                "t",
                "segment",
                "cmd_v",
                "cmd_w",
                "ekf_v",
                "ekf_wz",
                "ekf_yaw_u",
                "wheel_v",
                "wheel_wz",
                "wheel_yaw_u",
                "imu_wz",
                "imu_yaw_u",
                "err_yaw_ekf_imu",
                "err_yaw_wheel_imu",
                "err_wz_ekf_imu",
                "err_wz_wheel_imu",
            ],
        )
        self.writer.writeheader()

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.rows = []
        self.done = False
        self.last_print_sec = -1

        self.cmd_v = 0.0
        self.cmd_w = 0.0

        self.ekf_v = None
        self.ekf_wz = None
        self.ekf_yaw_raw = None
        self.ekf_yaw_u = None

        self.wheel_v = None
        self.wheel_wz = None
        self.wheel_yaw_raw = None
        self.wheel_yaw_u = None

        self.imu_wz = None
        self.imu_yaw_raw = None
        self.imu_yaw_u = None

        self.create_subscription(Odometry, "/odometry/filtered", self.cb_ekf, 50)
        self.create_subscription(Odometry, "/odom", self.cb_wheel, 50)
        self.create_subscription(Imu, "/camera/camera/imu_fixed", self.cb_imu, 200)
        self.create_subscription(
            TwistStamped, "/controller_server/RLController/desired_cmd", self.cb_cmd_ts, 50
        )
        self.create_subscription(Twist, "/cmd_vel", self.cb_cmd, 50)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.get_logger().info(f"csv: {self.csv_path}")
        self.get_logger().info("start: collect EKF/Wheel/IMU yaw metrics")

    def cb_cmd_ts(self, msg: TwistStamped):
        self.cmd_v = float(msg.twist.linear.x)
        self.cmd_w = float(msg.twist.angular.z)

    def cb_cmd(self, msg: Twist):
        self.cmd_v = float(msg.linear.x)
        self.cmd_w = float(msg.angular.z)

    def cb_ekf(self, msg: Odometry):
        self.ekf_v = float(msg.twist.twist.linear.x)
        self.ekf_wz = float(msg.twist.twist.angular.z)
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.ekf_yaw_raw, self.ekf_yaw_u = unwrap_step(self.ekf_yaw_raw, self.ekf_yaw_u, yaw)

    def cb_wheel(self, msg: Odometry):
        self.wheel_v = float(msg.twist.twist.linear.x)
        self.wheel_wz = float(msg.twist.twist.angular.z)
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.wheel_yaw_raw, self.wheel_yaw_u = unwrap_step(
            self.wheel_yaw_raw, self.wheel_yaw_u, yaw
        )

    def cb_imu(self, msg: Imu):
        self.imu_wz = float(msg.angular_velocity.z)
        q = msg.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.imu_yaw_raw, self.imu_yaw_u = unwrap_step(self.imu_yaw_raw, self.imu_yaw_u, yaw)

    def classify_segment(self, v_ref, w_ref):
        if abs(v_ref) <= self.v_thresh and abs(w_ref) >= self.w_thresh:
            return "rotate"
        if abs(v_ref) > self.v_thresh and abs(w_ref) <= self.w_thresh:
            return "straight"
        if abs(v_ref) <= self.v_thresh and abs(w_ref) < self.w_thresh:
            return "stop"
        return "mixed"

    def on_timer(self):
        if self.done:
            return

        required = [
            self.ekf_v,
            self.ekf_wz,
            self.ekf_yaw_u,
            self.wheel_v,
            self.wheel_wz,
            self.wheel_yaw_u,
            self.imu_wz,
            self.imu_yaw_u,
        ]
        if any(v is None for v in required):
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self.t0

        v_ref, w_ref = self.cmd_v, self.cmd_w
        if abs(v_ref) < 1e-6 and abs(w_ref) < 1e-6:
            v_ref, w_ref = self.wheel_v, self.wheel_wz

        seg = self.classify_segment(v_ref, w_ref)

        err_yaw_ekf_imu = wrap_pi(self.ekf_yaw_u - self.imu_yaw_u)
        err_yaw_wheel_imu = wrap_pi(self.wheel_yaw_u - self.imu_yaw_u)
        err_wz_ekf_imu = self.ekf_wz - self.imu_wz
        err_wz_wheel_imu = self.wheel_wz - self.imu_wz

        row = {
            "t": f"{t:.6f}",
            "segment": seg,
            "cmd_v": f"{self.cmd_v:.6f}",
            "cmd_w": f"{self.cmd_w:.6f}",
            "ekf_v": f"{self.ekf_v:.6f}",
            "ekf_wz": f"{self.ekf_wz:.6f}",
            "ekf_yaw_u": f"{self.ekf_yaw_u:.6f}",
            "wheel_v": f"{self.wheel_v:.6f}",
            "wheel_wz": f"{self.wheel_wz:.6f}",
            "wheel_yaw_u": f"{self.wheel_yaw_u:.6f}",
            "imu_wz": f"{self.imu_wz:.6f}",
            "imu_yaw_u": f"{self.imu_yaw_u:.6f}",
            "err_yaw_ekf_imu": f"{err_yaw_ekf_imu:.6f}",
            "err_yaw_wheel_imu": f"{err_yaw_wheel_imu:.6f}",
            "err_wz_ekf_imu": f"{err_wz_ekf_imu:.6f}",
            "err_wz_wheel_imu": f"{err_wz_wheel_imu:.6f}",
        }
        self.writer.writerow(row)
        self.fp.flush()

        self.rows.append(
            {
                "t": t,
                "segment": seg,
                "ekf_yaw_u": self.ekf_yaw_u,
                "wheel_yaw_u": self.wheel_yaw_u,
                "imu_yaw_u": self.imu_yaw_u,
                "err_yaw_ekf_imu": err_yaw_ekf_imu,
                "err_yaw_wheel_imu": err_yaw_wheel_imu,
                "err_wz_ekf_imu": err_wz_ekf_imu,
                "err_wz_wheel_imu": err_wz_wheel_imu,
            }
        )

        sec_i = int(t)
        if sec_i % 5 == 0 and sec_i != self.last_print_sec:
            self.last_print_sec = sec_i
            self.get_logger().info(
                f"t={t:6.1f}s seg={seg:8s} | "
                f"wz(err) ekf-imu={err_wz_ekf_imu:+.4f}, wheel-imu={err_wz_wheel_imu:+.4f}"
            )

        if t >= self.duration:
            self.finish_and_shutdown()

    def finish_and_shutdown(self):
        if self.done:
            return
        self.done = True
        self.fp.close()
        self.print_summary()
        self.get_logger().info(f"done. csv: {self.csv_path}")
        if rclpy.ok():
            rclpy.shutdown()

    def print_summary(self):
        if len(self.rows) < 2:
            self.get_logger().warn("rows too small")
            return

        rot = [r for r in self.rows if r["segment"] == "rotate"]
        if len(rot) < 2:
            self.get_logger().warn("rotate segment too small")
            return

        def mae_abs(key, rows):
            return statistics.fmean(abs(r[key]) for r in rows)

        d_imu = rot[-1]["imu_yaw_u"] - rot[0]["imu_yaw_u"]
        d_ekf = rot[-1]["ekf_yaw_u"] - rot[0]["ekf_yaw_u"]
        d_wheel = rot[-1]["wheel_yaw_u"] - rot[0]["wheel_yaw_u"]

        ratio_ekf = d_ekf / d_imu if abs(d_imu) > 1e-6 else float("nan")
        ratio_wheel = d_wheel / d_imu if abs(d_imu) > 1e-6 else float("nan")

        self.get_logger().info("========== SUMMARY (ROTATE) ==========")
        self.get_logger().info(f"rotate samples: {len(rot)}")
        self.get_logger().info(
            f"MAE |yaw(ekf-imu)|  : {math.degrees(mae_abs('err_yaw_ekf_imu', rot)):.3f} deg"
        )
        self.get_logger().info(
            f"MAE |yaw(wheel-imu)|: {math.degrees(mae_abs('err_yaw_wheel_imu', rot)):.3f} deg"
        )
        self.get_logger().info(f"MAE |wz(ekf-imu)|   : {mae_abs('err_wz_ekf_imu', rot):.4f} rad/s")
        self.get_logger().info(
            f"MAE |wz(wheel-imu)| : {mae_abs('err_wz_wheel_imu', rot):.4f} rad/s"
        )
        self.get_logger().info(f"Dyaw imu   : {math.degrees(d_imu):.2f} deg")
        self.get_logger().info(
            f"Dyaw ekf   : {math.degrees(d_ekf):.2f} deg (ekf/imu={ratio_ekf:.3f})"
        )
        self.get_logger().info(
            f"Dyaw wheel : {math.degrees(d_wheel):.2f} deg (wheel/imu={ratio_wheel:.3f})"
        )

        if not math.isnan(ratio_wheel) and abs(ratio_wheel - 1.0) > 0.15:
            self.get_logger().warn(
                "wheel yaw reliability seems low: lower odom yaw weight in EKF"
            )
        if not math.isnan(ratio_ekf) and abs(ratio_ekf - 1.0) > 0.10:
            self.get_logger().warn(
                "EKF yaw deviation is high: increase imu vyaw weight, lower odom yaw weight"
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=180.0, help="record sec")
    parser.add_argument("--rate", type=float, default=20.0, help="log Hz")
    parser.add_argument("--v-thresh", type=float, default=0.03, help="straight/stop threshold")
    parser.add_argument("--w-thresh", type=float, default=0.08, help="rotate threshold")
    parser.add_argument(
        "--csv",
        type=str,
        default=f"~/ca_ws/logs/ekf_yaw_probe_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    args = parser.parse_args()

    rclpy.init()
    node = EkfYawProbe(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish_and_shutdown()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
