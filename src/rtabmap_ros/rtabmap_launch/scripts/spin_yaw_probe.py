#!/usr/bin/env python3
"""
spin_yaw_probe.py

Record local yaw behavior during pure-spin tests.

The node samples:
- /odometry/filtered yaw and angular velocity
- /camera/camera/imu_fixed angular velocity

It writes append-only JSONL records so that later analysis can use cursor-based
queries instead of offset paging.

Record types:
- meta:        probe configuration and file paths
- sample:      periodic combined odom/imu sample
- spin_enter:  pure-spin session start
- spin_exit:   pure-spin session summary
- shutdown:    final probe summary
"""

import json
import math
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def clamp01(value: float) -> float:
    return clamp(value, 0.0, 1.0)


def ema(current: float, target: float, dt: float, tau: float) -> float:
    if tau <= 1.0e-6:
        return target
    alpha = 1.0 - math.exp(-dt / tau)
    return current + alpha * (target - current)


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def safe_round(value: Optional[float], digits: int = 6):
    if value is None:
        return None
    return round(float(value), digits)


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def format_stamp_ns(stamp_ns: int) -> str:
    sec = stamp_ns // 1_000_000_000
    nsec = stamp_ns % 1_000_000_000
    return f"{sec}.{nsec:09d}"


def pose_from_transform(transform):
    return (
        float(transform.translation.x),
        float(transform.translation.y),
        yaw_from_quaternion(transform.rotation),
    )


class SpinYawProbe(Node):
    def __init__(self):
        super().__init__("spin_yaw_probe")

        self.odom_topic = self.declare_parameter("odom_topic", "/odometry/filtered").value
        self.imu_topic = self.declare_parameter("imu_topic", "/camera/camera/imu_fixed").value
        self.tf_parent_frame = self.declare_parameter("tf_parent_frame", "odom").value
        self.tf_child_frame = self.declare_parameter("tf_child_frame", "base_link").value
        self.tf_lookup_timeout_sec = float(
            self.declare_parameter("tf_lookup_timeout_sec", 0.02).value
        )
        self.trial_id = self.declare_parameter("trial_id", "").value
        self.output_path_param = self.declare_parameter("output_path", "").value
        self.tick_hz = float(self.declare_parameter("tick_hz", 20.0).value)
        self.log_period_sec = float(self.declare_parameter("log_period_sec", 1.0).value)

        # Pure-spin detector parameters intentionally mirror the stabilizers so
        # the probe and the runtime mitigation observe the same spin window.
        self.wz_filter_tau_sec = float(self.declare_parameter("wz_filter_tau_sec", 0.06).value)
        self.speed_filter_tau_sec = float(
            self.declare_parameter("speed_filter_tau_sec", 0.12).value
        )
        self.spin_wz_start = float(self.declare_parameter("spin_wz_start", 0.08).value)
        self.spin_wz_full = float(self.declare_parameter("spin_wz_full", 0.18).value)
        self.spin_speed_quiet = float(self.declare_parameter("spin_speed_quiet", 0.05).value)
        self.spin_duration_ref_sec = float(
            self.declare_parameter("spin_duration_ref_sec", 0.80).value
        )
        self.spin_yaw_ref_rad = float(self.declare_parameter("spin_yaw_ref_rad", 0.20).value)
        self.spin_decay_sec = float(self.declare_parameter("spin_decay_sec", 0.80).value)
        self.score_fast_weight = float(self.declare_parameter("score_fast_weight", 0.75).value)
        self.score_slow_weight = float(self.declare_parameter("score_slow_weight", 0.25).value)
        self.score_rise_tau_sec = float(
            self.declare_parameter("score_rise_tau_sec", 0.05).value
        )
        self.score_fall_tau_sec = float(
            self.declare_parameter("score_fall_tau_sec", 0.50).value
        )

        self.session_enter_score = float(
            self.declare_parameter("session_enter_score", 0.55).value
        )
        self.session_exit_score = float(
            self.declare_parameter("session_exit_score", 0.20).value
        )
        self.session_exit_hold_sec = float(
            self.declare_parameter("session_exit_hold_sec", 0.25).value
        )

        self.trial_id = self.trial_id or datetime.now().strftime("spin_yaw_trial_%Y%m%d_%H%M%S")
        self.output_path = self._resolve_output_path(self.output_path_param)
        self.summary_path = self.output_path.with_suffix(".summary.json")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_fp = self.output_path.open("w", encoding="utf-8")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.seq = 0
        self.started_at_ns = self.get_clock().now().nanoseconds
        self.last_status_log_ns = self.started_at_ns
        self.last_tick_ns = self.started_at_ns
        self.last_tf_warn_ns = self.started_at_ns

        # Latest odom sample.
        self.odom_frame_id = ""
        self.odom_stamp_ns = 0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_speed_raw = 0.0
        self.odom_speed_f = 0.0
        self.odom_wz = 0.0
        self.odom_yaw_raw_prev = None
        self.odom_yaw_unwrapped = None

        # Latest IMU sample.
        self.imu_frame_id = ""
        self.imu_stamp_ns = 0
        self.imu_wz_raw = 0.0
        self.imu_wz_f = 0.0
        self.imu_yaw_integral_raw = 0.0
        self.imu_yaw_integral_f = 0.0
        self.last_imu_stamp_ns = None

        # Latest TF sample.
        self.tf_stamp_ns = 0
        self.tf_x = 0.0
        self.tf_y = 0.0
        self.tf_yaw_raw_prev = None
        self.tf_yaw_unwrapped = None
        self.tf_lookup_ok = False

        # Spin detector state.
        self.spin_duration = 0.0
        self.spin_yaw_accum = 0.0
        self.spin_score = 0.0
        self.spin_fast_term = 0.0
        self.spin_slow_term = 0.0

        # Session state.
        self.session_active = False
        self.session_index = 0
        self.session_id = None
        self.session_enter_stamp_ns = None
        self.session_exit_hold_accum = 0.0
        self.session_baseline_odom_yaw = None
        self.session_baseline_imu_yaw_raw = None
        self.session_baseline_imu_yaw_f = None
        self.session_baseline_tf_yaw = None
        self.session_sample_count = 0
        self.session_peak_score = 0.0
        self.session_peak_abs_wz = 0.0
        self.session_max_speed = 0.0
        self.session_sum_abs_odom_wz = 0.0
        self.session_sum_abs_imu_wz = 0.0
        self.session_sum_abs_pose_tf_yaw_diff = 0.0
        self.session_peak_abs_pose_tf_yaw_diff = 0.0
        self.session_sum_pose_tf_pos_err = 0.0
        self.session_peak_pose_tf_pos_err = 0.0
        self.session_summaries = []

        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 50)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 200)
        self.create_timer(1.0 / max(self.tick_hz, 1.0), self._tick)

        self._write_event(
            {
                "record_type": "meta",
                "trial_id": self.trial_id,
                "output_path": str(self.output_path),
                "summary_path": str(self.summary_path),
                "odom_topic": self.odom_topic,
                "imu_topic": self.imu_topic,
                "tf_parent_frame": self.tf_parent_frame,
                "tf_child_frame": self.tf_child_frame,
                "tf_lookup_timeout_sec": self.tf_lookup_timeout_sec,
                "tick_hz": self.tick_hz,
                "session_enter_score": self.session_enter_score,
                "session_exit_score": self.session_exit_score,
                "session_exit_hold_sec": self.session_exit_hold_sec,
                "spin_wz_start": self.spin_wz_start,
                "spin_wz_full": self.spin_wz_full,
                "spin_speed_quiet": self.spin_speed_quiet,
            }
        )

        self.get_logger().info(
            f"[SpinYawProbe] trial_id={self.trial_id} output={self.output_path} "
            f"summary={self.summary_path}"
        )

    def _resolve_output_path(self, output_path: str) -> Path:
        if output_path:
            return Path(output_path).expanduser()
        filename = f"{self.trial_id}.jsonl"
        return Path("~/ca_ws/logs").expanduser() / filename

    def _write_event(self, event: dict):
        event = dict(event)
        event["seq"] = self.seq
        self.seq += 1
        json.dump(event, self.output_fp, ensure_ascii=True, sort_keys=True)
        self.output_fp.write("\n")
        self.output_fp.flush()

    def _odom_cb(self, msg: Odometry):
        self.odom_frame_id = msg.header.frame_id
        self.odom_stamp_ns = stamp_to_ns(msg.header.stamp)
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)
        self.odom_speed_raw = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.odom_wz = float(msg.twist.twist.angular.z)

        yaw_raw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.odom_yaw_raw_prev is None or self.odom_yaw_unwrapped is None:
            self.odom_yaw_raw_prev = yaw_raw
            self.odom_yaw_unwrapped = yaw_raw
        else:
            delta = wrap_pi(yaw_raw - self.odom_yaw_raw_prev)
            self.odom_yaw_raw_prev = yaw_raw
            self.odom_yaw_unwrapped += delta

    def _imu_cb(self, msg: Imu):
        self.imu_frame_id = msg.header.frame_id
        self.imu_stamp_ns = stamp_to_ns(msg.header.stamp)
        self.imu_wz_raw = float(msg.angular_velocity.z)

        dt = 0.0
        if self.last_imu_stamp_ns is not None and self.imu_stamp_ns > self.last_imu_stamp_ns:
            dt = (self.imu_stamp_ns - self.last_imu_stamp_ns) / 1.0e9
            if dt > 0.5:
                dt = 0.0
        self.last_imu_stamp_ns = self.imu_stamp_ns

        if dt > 0.0:
            self.imu_wz_f = ema(self.imu_wz_f, self.imu_wz_raw, dt, self.wz_filter_tau_sec)
            self.imu_yaw_integral_raw += self.imu_wz_raw * dt
            self.imu_yaw_integral_f += self.imu_wz_f * dt
        else:
            self.imu_wz_f = self.imu_wz_raw

    def _lookup_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.tf_parent_frame,
                self.tf_child_frame,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_tf_warn_ns >= int(1.0e9):
                self.last_tf_warn_ns = now_ns
                self.get_logger().warning(
                    f"[SpinYawProbe] waiting for TF "
                    f"{self.tf_parent_frame}->{self.tf_child_frame}: {exc}"
                )
            self.tf_lookup_ok = False
            return

        self.tf_lookup_ok = True
        self.tf_stamp_ns = stamp_to_ns(transform.header.stamp)
        self.tf_x, self.tf_y, tf_yaw_raw = pose_from_transform(transform.transform)
        if self.tf_yaw_raw_prev is None or self.tf_yaw_unwrapped is None:
            self.tf_yaw_raw_prev = tf_yaw_raw
            self.tf_yaw_unwrapped = tf_yaw_raw
        else:
            delta = wrap_pi(tf_yaw_raw - self.tf_yaw_raw_prev)
            self.tf_yaw_raw_prev = tf_yaw_raw
            self.tf_yaw_unwrapped += delta

    def _update_spin_score(self, dt: float):
        self.odom_speed_f = ema(
            self.odom_speed_f, self.odom_speed_raw, dt, self.speed_filter_tau_sec
        )

        abs_wz = abs(self.imu_wz_f)
        wz_term = clamp01(
            (abs_wz - self.spin_wz_start) / max(self.spin_wz_full - self.spin_wz_start, 1.0e-6)
        )
        quiet_term = 1.0 - clamp01(self.odom_speed_f / max(self.spin_speed_quiet, 1.0e-6))
        fast_term = wz_term * quiet_term

        if fast_term > 0.0:
            self.spin_duration += dt
            self.spin_yaw_accum += abs_wz * dt
        else:
            decay = math.exp(-dt / max(self.spin_decay_sec, 1.0e-6))
            self.spin_duration *= decay
            self.spin_yaw_accum *= decay

        dur_term = clamp01(self.spin_duration / max(self.spin_duration_ref_sec, 1.0e-6))
        yaw_term = clamp01(self.spin_yaw_accum / max(self.spin_yaw_ref_rad, 1.0e-6))
        slow_term = max(dur_term, yaw_term)

        target = clamp01(self.score_fast_weight * fast_term + self.score_slow_weight * slow_term)
        tau = self.score_rise_tau_sec if target >= self.spin_score else self.score_fall_tau_sec
        self.spin_score = ema(self.spin_score, target, dt, tau)
        self.spin_fast_term = fast_term
        self.spin_slow_term = slow_term

    def _begin_session(self, stamp_ns: int):
        self.session_active = True
        self.session_index += 1
        self.session_id = f"{self.trial_id}_spin_{self.session_index:03d}"
        self.session_enter_stamp_ns = stamp_ns
        self.session_exit_hold_accum = 0.0
        self.session_baseline_odom_yaw = self.odom_yaw_unwrapped
        self.session_baseline_imu_yaw_raw = self.imu_yaw_integral_raw
        self.session_baseline_imu_yaw_f = self.imu_yaw_integral_f
        self.session_baseline_tf_yaw = self.tf_yaw_unwrapped if self.tf_lookup_ok else None
        self.session_sample_count = 0
        self.session_peak_score = self.spin_score
        self.session_peak_abs_wz = abs(self.imu_wz_raw)
        self.session_max_speed = self.odom_speed_raw
        self.session_sum_abs_odom_wz = 0.0
        self.session_sum_abs_imu_wz = 0.0
        self.session_sum_abs_pose_tf_yaw_diff = 0.0
        self.session_peak_abs_pose_tf_yaw_diff = 0.0
        self.session_sum_pose_tf_pos_err = 0.0
        self.session_peak_pose_tf_pos_err = 0.0

        self._write_event(
            {
                "record_type": "spin_enter",
                "trial_id": self.trial_id,
                "session_index": self.session_index,
                "session_id": self.session_id,
                "stamp_ns": stamp_ns,
                "stamp": format_stamp_ns(stamp_ns),
                "spin_score": safe_round(self.spin_score, 6),
                "imu_wz_raw": safe_round(self.imu_wz_raw, 6),
                "imu_wz_f": safe_round(self.imu_wz_f, 6),
                "odom_speed": safe_round(self.odom_speed_raw, 6),
                "odom_wz": safe_round(self.odom_wz, 6),
                "odom_x": safe_round(self.odom_x, 6),
                "odom_y": safe_round(self.odom_y, 6),
                "odom_yaw_unwrapped": safe_round(self.odom_yaw_unwrapped, 6),
                "imu_yaw_integral_raw": safe_round(self.imu_yaw_integral_raw, 6),
                "imu_yaw_integral_f": safe_round(self.imu_yaw_integral_f, 6),
                "tf_lookup_ok": self.tf_lookup_ok,
                "tf_stamp_ns": self.tf_stamp_ns if self.tf_lookup_ok else None,
                "tf_stamp": format_stamp_ns(self.tf_stamp_ns) if self.tf_lookup_ok else None,
                "tf_x": safe_round(self.tf_x, 6) if self.tf_lookup_ok else None,
                "tf_y": safe_round(self.tf_y, 6) if self.tf_lookup_ok else None,
                "tf_yaw_unwrapped": safe_round(self.tf_yaw_unwrapped, 6),
            }
        )
        self.get_logger().warning(
            f"[SpinYawProbe] session_enter id={self.session_id} "
            f"score={self.spin_score:.3f} wz={self.imu_wz_f:.3f} speed={self.odom_speed_f:.3f}"
        )

    def _end_session(self, stamp_ns: int, reason: str):
        if not self.session_active or self.session_enter_stamp_ns is None:
            return

        odom_delta = None
        if self.odom_yaw_unwrapped is not None and self.session_baseline_odom_yaw is not None:
            odom_delta = self.odom_yaw_unwrapped - self.session_baseline_odom_yaw

        imu_delta_raw = None
        if self.session_baseline_imu_yaw_raw is not None:
            imu_delta_raw = self.imu_yaw_integral_raw - self.session_baseline_imu_yaw_raw

        imu_delta_f = None
        if self.session_baseline_imu_yaw_f is not None:
            imu_delta_f = self.imu_yaw_integral_f - self.session_baseline_imu_yaw_f

        tf_delta = None
        if self.tf_lookup_ok and self.tf_yaw_unwrapped is not None and self.session_baseline_tf_yaw is not None:
            tf_delta = self.tf_yaw_unwrapped - self.session_baseline_tf_yaw

        duration_sec = (stamp_ns - self.session_enter_stamp_ns) / 1.0e9
        ratio_odom_over_imu_raw = None
        if imu_delta_raw is not None and abs(imu_delta_raw) > 1.0e-6 and odom_delta is not None:
            ratio_odom_over_imu_raw = odom_delta / imu_delta_raw

        ratio_tf_over_imu_raw = None
        if imu_delta_raw is not None and abs(imu_delta_raw) > 1.0e-6 and tf_delta is not None:
            ratio_tf_over_imu_raw = tf_delta / imu_delta_raw

        mean_abs_odom_wz = None
        mean_abs_imu_wz = None
        mean_abs_pose_tf_yaw_diff = None
        mean_pose_tf_pos_err = None
        if self.session_sample_count > 0:
            mean_abs_odom_wz = self.session_sum_abs_odom_wz / self.session_sample_count
            mean_abs_imu_wz = self.session_sum_abs_imu_wz / self.session_sample_count
            mean_abs_pose_tf_yaw_diff = (
                self.session_sum_abs_pose_tf_yaw_diff / self.session_sample_count
            )
            mean_pose_tf_pos_err = self.session_sum_pose_tf_pos_err / self.session_sample_count

        pose_minus_tf_delta = None
        if odom_delta is not None and tf_delta is not None:
            pose_minus_tf_delta = odom_delta - tf_delta

        summary = {
            "session_index": self.session_index,
            "session_id": self.session_id,
            "enter_stamp_ns": self.session_enter_stamp_ns,
            "exit_stamp_ns": stamp_ns,
            "duration_sec": safe_round(duration_sec, 6),
            "sample_count": self.session_sample_count,
            "peak_spin_score": safe_round(self.session_peak_score, 6),
            "peak_abs_imu_wz": safe_round(self.session_peak_abs_wz, 6),
            "max_speed": safe_round(self.session_max_speed, 6),
            "mean_abs_odom_wz": safe_round(mean_abs_odom_wz, 6),
            "mean_abs_imu_wz": safe_round(mean_abs_imu_wz, 6),
            "odom_delta_yaw_rad": safe_round(odom_delta, 6),
            "odom_delta_yaw_deg": safe_round(
                math.degrees(odom_delta) if odom_delta is not None else None, 3
            ),
            "tf_delta_yaw_rad": safe_round(tf_delta, 6),
            "tf_delta_yaw_deg": safe_round(
                math.degrees(tf_delta) if tf_delta is not None else None, 3
            ),
            "imu_delta_yaw_raw_rad": safe_round(imu_delta_raw, 6),
            "imu_delta_yaw_raw_deg": safe_round(
                math.degrees(imu_delta_raw) if imu_delta_raw is not None else None, 3
            ),
            "imu_delta_yaw_f_rad": safe_round(imu_delta_f, 6),
            "imu_delta_yaw_f_deg": safe_round(
                math.degrees(imu_delta_f) if imu_delta_f is not None else None, 3
            ),
            "ratio_odom_over_imu_raw": safe_round(ratio_odom_over_imu_raw, 6),
            "ratio_tf_over_imu_raw": safe_round(ratio_tf_over_imu_raw, 6),
            "pose_minus_tf_delta_yaw_rad": safe_round(pose_minus_tf_delta, 6),
            "pose_minus_tf_delta_yaw_deg": safe_round(
                math.degrees(pose_minus_tf_delta) if pose_minus_tf_delta is not None else None, 3
            ),
            "mean_abs_pose_tf_yaw_diff_rad": safe_round(mean_abs_pose_tf_yaw_diff, 6),
            "mean_abs_pose_tf_yaw_diff_deg": safe_round(
                math.degrees(mean_abs_pose_tf_yaw_diff)
                if mean_abs_pose_tf_yaw_diff is not None
                else None,
                3,
            ),
            "peak_abs_pose_tf_yaw_diff_rad": safe_round(
                self.session_peak_abs_pose_tf_yaw_diff, 6
            ),
            "peak_abs_pose_tf_yaw_diff_deg": safe_round(
                math.degrees(self.session_peak_abs_pose_tf_yaw_diff), 3
            ),
            "mean_pose_tf_pos_err_m": safe_round(mean_pose_tf_pos_err, 6),
            "peak_pose_tf_pos_err_m": safe_round(self.session_peak_pose_tf_pos_err, 6),
            "exit_reason": reason,
        }
        self.session_summaries.append(summary)

        self._write_event(
            {
                "record_type": "spin_exit",
                "trial_id": self.trial_id,
                "stamp_ns": stamp_ns,
                "stamp": format_stamp_ns(stamp_ns),
                **summary,
            }
        )

        self.get_logger().warning(
            f"[SpinYawProbe] session_exit id={self.session_id} reason={reason} "
            f"duration={duration_sec:.2f}s odom_dyaw={math.degrees(odom_delta or 0.0):.2f}deg "
            f"tf_dyaw={math.degrees(tf_delta or 0.0):.2f}deg "
            f"imu_dyaw={math.degrees(imu_delta_raw or 0.0):.2f}deg "
            f"ratio_pose={ratio_odom_over_imu_raw if ratio_odom_over_imu_raw is not None else float('nan'):.3f} "
            f"ratio_tf={ratio_tf_over_imu_raw if ratio_tf_over_imu_raw is not None else float('nan'):.3f} "
            f"pose_tf={math.degrees(pose_minus_tf_delta or 0.0):+.2f}deg"
        )

        self.session_active = False
        self.session_id = None
        self.session_enter_stamp_ns = None
        self.session_exit_hold_accum = 0.0
        self.session_baseline_odom_yaw = None
        self.session_baseline_imu_yaw_raw = None
        self.session_baseline_imu_yaw_f = None
        self.session_baseline_tf_yaw = None
        self.session_sample_count = 0
        self.session_peak_score = 0.0
        self.session_peak_abs_wz = 0.0
        self.session_max_speed = 0.0
        self.session_sum_abs_odom_wz = 0.0
        self.session_sum_abs_imu_wz = 0.0
        self.session_sum_abs_pose_tf_yaw_diff = 0.0
        self.session_peak_abs_pose_tf_yaw_diff = 0.0
        self.session_sum_pose_tf_pos_err = 0.0
        self.session_peak_pose_tf_pos_err = 0.0

    def _active_session_odom_delta(self):
        if not self.session_active or self.session_baseline_odom_yaw is None:
            return None
        if self.odom_yaw_unwrapped is None:
            return None
        return self.odom_yaw_unwrapped - self.session_baseline_odom_yaw

    def _active_session_imu_delta_raw(self):
        if not self.session_active or self.session_baseline_imu_yaw_raw is None:
            return None
        return self.imu_yaw_integral_raw - self.session_baseline_imu_yaw_raw

    def _active_session_imu_delta_f(self):
        if not self.session_active or self.session_baseline_imu_yaw_f is None:
            return None
        return self.imu_yaw_integral_f - self.session_baseline_imu_yaw_f

    def _active_session_tf_delta(self):
        if not self.session_active or self.session_baseline_tf_yaw is None:
            return None
        if not self.tf_lookup_ok or self.tf_yaw_unwrapped is None:
            return None
        return self.tf_yaw_unwrapped - self.session_baseline_tf_yaw

    def _tick(self):
        if self.odom_yaw_unwrapped is None or self.imu_stamp_ns == 0 or self.odom_stamp_ns == 0:
            return

        now_ns = self.get_clock().now().nanoseconds
        dt = max((now_ns - self.last_tick_ns) / 1.0e9, 1.0e-3)
        self.last_tick_ns = now_ns

        self._lookup_tf()
        self._update_spin_score(dt)

        event_stamp_ns = self.odom_stamp_ns if self.odom_stamp_ns > 0 else now_ns

        if self.session_active:
            self.session_sample_count += 1
            self.session_peak_score = max(self.session_peak_score, self.spin_score)
            self.session_peak_abs_wz = max(self.session_peak_abs_wz, abs(self.imu_wz_raw))
            self.session_max_speed = max(self.session_max_speed, self.odom_speed_raw)
            self.session_sum_abs_odom_wz += abs(self.odom_wz)
            self.session_sum_abs_imu_wz += abs(self.imu_wz_raw)
            pose_tf_yaw_diff = None
            if self.tf_lookup_ok and self.tf_yaw_unwrapped is not None:
                pose_tf_yaw_diff = self.odom_yaw_unwrapped - self.tf_yaw_unwrapped
                self.session_sum_abs_pose_tf_yaw_diff += abs(pose_tf_yaw_diff)
                self.session_peak_abs_pose_tf_yaw_diff = max(
                    self.session_peak_abs_pose_tf_yaw_diff, abs(pose_tf_yaw_diff)
                )
                pose_tf_pos_err = math.hypot(self.odom_x - self.tf_x, self.odom_y - self.tf_y)
                self.session_sum_pose_tf_pos_err += pose_tf_pos_err
                self.session_peak_pose_tf_pos_err = max(
                    self.session_peak_pose_tf_pos_err, pose_tf_pos_err
                )

            if self.spin_score <= self.session_exit_score:
                self.session_exit_hold_accum += dt
            else:
                self.session_exit_hold_accum = 0.0

            if self.session_exit_hold_accum >= self.session_exit_hold_sec:
                self._end_session(event_stamp_ns, "spin_score_relaxed")
        else:
            if self.spin_score >= self.session_enter_score:
                self._begin_session(event_stamp_ns)

        sample_event = {
            "record_type": "sample",
            "trial_id": self.trial_id,
            "stamp_ns": event_stamp_ns,
            "stamp": format_stamp_ns(event_stamp_ns),
            "odom_stamp_ns": self.odom_stamp_ns,
            "imu_stamp_ns": self.imu_stamp_ns,
            "odom_age_sec": safe_round((now_ns - self.odom_stamp_ns) / 1.0e9, 6),
            "imu_age_sec": safe_round((now_ns - self.imu_stamp_ns) / 1.0e9, 6),
            "tf_age_sec": safe_round((now_ns - self.tf_stamp_ns) / 1.0e9, 6)
            if self.tf_lookup_ok and self.tf_stamp_ns > 0
            else None,
            "odom_frame_id": self.odom_frame_id,
            "imu_frame_id": self.imu_frame_id,
            "tf_lookup_ok": self.tf_lookup_ok,
            "tf_parent_frame": self.tf_parent_frame,
            "tf_child_frame": self.tf_child_frame,
            "tf_stamp_ns": self.tf_stamp_ns if self.tf_lookup_ok else None,
            "session_active": self.session_active,
            "session_index": self.session_index if self.session_active else None,
            "session_id": self.session_id,
            "spin_score": safe_round(self.spin_score, 6),
            "spin_fast_term": safe_round(self.spin_fast_term, 6),
            "spin_slow_term": safe_round(self.spin_slow_term, 6),
            "spin_duration": safe_round(self.spin_duration, 6),
            "spin_yaw_accum": safe_round(self.spin_yaw_accum, 6),
            "odom_speed": safe_round(self.odom_speed_raw, 6),
            "odom_speed_f": safe_round(self.odom_speed_f, 6),
            "odom_x": safe_round(self.odom_x, 6),
            "odom_y": safe_round(self.odom_y, 6),
            "odom_wz": safe_round(self.odom_wz, 6),
            "odom_yaw_unwrapped": safe_round(self.odom_yaw_unwrapped, 6),
            "odom_session_delta_yaw": safe_round(self._active_session_odom_delta(), 6),
            "tf_x": safe_round(self.tf_x, 6) if self.tf_lookup_ok else None,
            "tf_y": safe_round(self.tf_y, 6) if self.tf_lookup_ok else None,
            "tf_yaw_unwrapped": safe_round(self.tf_yaw_unwrapped, 6),
            "tf_session_delta_yaw": safe_round(self._active_session_tf_delta(), 6),
            "pose_tf_yaw_diff": safe_round(
                (self.odom_yaw_unwrapped - self.tf_yaw_unwrapped)
                if self.tf_lookup_ok and self.tf_yaw_unwrapped is not None
                else None,
                6,
            ),
            "pose_tf_pos_err": safe_round(
                math.hypot(self.odom_x - self.tf_x, self.odom_y - self.tf_y)
                if self.tf_lookup_ok
                else None,
                6,
            ),
            "imu_wz_raw": safe_round(self.imu_wz_raw, 6),
            "imu_wz_f": safe_round(self.imu_wz_f, 6),
            "imu_yaw_integral_raw": safe_round(self.imu_yaw_integral_raw, 6),
            "imu_yaw_integral_f": safe_round(self.imu_yaw_integral_f, 6),
            "imu_session_delta_yaw_raw": safe_round(self._active_session_imu_delta_raw(), 6),
            "imu_session_delta_yaw_f": safe_round(self._active_session_imu_delta_f(), 6),
        }
        self._write_event(sample_event)

        if now_ns - self.last_status_log_ns >= int(self.log_period_sec * 1.0e9):
            self.last_status_log_ns = now_ns
            self.get_logger().info(
                f"[SpinYawProbe] active={int(self.session_active)} "
                f"score={self.spin_score:.3f} wz={self.imu_wz_f:.3f} "
                f"speed={self.odom_speed_f:.3f} "
                f"odom_dyaw={safe_round(self._active_session_odom_delta(), 4)} "
                f"tf_dyaw={safe_round(self._active_session_tf_delta(), 4)} "
                f"imu_dyaw={safe_round(self._active_session_imu_delta_raw(), 4)} "
                f"pose_tf={safe_round(sample_event['pose_tf_yaw_diff'], 4)}"
            )

    def shutdown(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.session_active:
            event_stamp_ns = self.odom_stamp_ns if self.odom_stamp_ns > 0 else now_ns
            self._end_session(event_stamp_ns, "shutdown")

        self._write_event(
            {
                "record_type": "shutdown",
                "trial_id": self.trial_id,
                "stamp_ns": now_ns,
                "stamp": format_stamp_ns(now_ns),
                "session_count": len(self.session_summaries),
                "output_path": str(self.output_path),
                "summary_path": str(self.summary_path),
            }
        )

        summary = {
            "trial_id": self.trial_id,
            "output_path": str(self.output_path),
            "summary_path": str(self.summary_path),
            "started_at_ns": self.started_at_ns,
            "stopped_at_ns": now_ns,
            "record_count": self.seq,
            "session_count": len(self.session_summaries),
            "sessions": self.session_summaries,
        }
        self.summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=True), "utf-8")
        self.output_fp.close()


def main(args=None):
    rclpy.init(args=args)
    node = SpinYawProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
