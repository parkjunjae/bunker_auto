#!/usr/bin/env python3
"""
map_tf_stabilizer.py

RTAB-Map raw map->odom TF는 그대로 유지하고, 그 위에 map_stable->map
안정화 계층을 추가한다.

핵심 구조:
1. RTAB-Map raw TF:          map -> odom
2. EKF local odom TF:        odom -> base_link
3. Stabilizer filtered TF:   map_stable -> map

따라서 Nav2/RViz가 map_stable을 global frame으로 보면, 실제 global chain은
다음과 같이 된다.

    map_stable -> map -> odom -> base_link

설계 원칙:
1. raw map->odom TF는 끄지 않는다.
2. pure spin 중에도 correction을 hard-freeze 하지 않는다.
3. 미반영 correction을 debt/backlog로 저장하지 않는다.
4. 매 주기 최신 raw TF만 읽고, 그 변화량 일부만 반영한다.
5. pure spin일수록 map_stable->map correction authority를 낮춘다.
6. 출력 yaw/xy 변화에는 rate limit를 걸어 step jump를 억제한다.
7. pure spin이 끝나면 map_stable->map을 천천히 identity로 복귀시킨다.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def clamp01(value):
    return clamp(value, 0.0, 1.0)


def lerp(a, b, t):
    return a + (b - a) * clamp01(t)


def angle_wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def ema(current, target, dt, tau):
    if tau <= 1.0e-6:
        return target
    alpha = 1.0 - math.exp(-dt / tau)
    return current + alpha * (target - current)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def pose_compose(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    ca = math.cos(ayaw)
    sa = math.sin(ayaw)
    return (
        ax + ca * bx - sa * by,
        ay + sa * bx + ca * by,
        angle_wrap(ayaw + byaw),
    )


def pose_inverse(pose):
    x, y, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        -(c * x + s * y),
        s * x - c * y,
        angle_wrap(-yaw),
    )


def pose_delta(prev_pose, next_pose):
    return pose_compose(pose_inverse(prev_pose), next_pose)


def clamp_vector_norm(x, y, max_norm):
    norm = math.hypot(x, y)
    if norm <= max_norm or norm <= 1.0e-9:
        return x, y
    scale = max_norm / norm
    return x * scale, y * scale


def pose_from_transform(transform):
    return (
        transform.translation.x,
        transform.translation.y,
        yaw_from_quaternion(transform.rotation),
    )


class MapTfStabilizer(Node):
    def __init__(self):
        super().__init__('map_tf_stabilizer')

        # Frames / inputs
        self.stable_map_frame_id = self.declare_parameter('stable_map_frame_id', 'map_stable').value
        self.map_frame_id = self.declare_parameter('map_frame_id', 'map').value
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value
        self.imu_topic = self.declare_parameter('imu_topic', '/camera/camera/imu_fixed').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odometry/filtered').value
        self.publish_hz = float(self.declare_parameter('publish_hz', 20.0).value)
        self.tf_lookup_timeout_sec = float(
            self.declare_parameter('tf_lookup_timeout_sec', 0.02).value
        )

        # Filter the motion evidence just enough to remove sample jitter.
        self.wz_filter_tau_sec = float(self.declare_parameter('wz_filter_tau_sec', 0.06).value)
        self.speed_filter_tau_sec = float(self.declare_parameter('speed_filter_tau_sec', 0.12).value)

        # Pure-spin score:
        # - use IMU wz as the primary spin evidence
        # - use odom speed as the "pure" term
        # - duration/yaw accumulation only strengthen confidence, they do not
        #   open the gate by themselves
        self.spin_wz_start = float(self.declare_parameter('spin_wz_start', 0.08).value)
        self.spin_wz_full = float(self.declare_parameter('spin_wz_full', 0.18).value)
        self.spin_speed_quiet = float(self.declare_parameter('spin_speed_quiet', 0.05).value)
        self.spin_duration_ref_sec = float(
            self.declare_parameter('spin_duration_ref_sec', 0.80).value
        )
        self.spin_yaw_ref_rad = float(self.declare_parameter('spin_yaw_ref_rad', 0.20).value)
        self.spin_decay_sec = float(self.declare_parameter('spin_decay_sec', 0.80).value)
        self.score_fast_weight = float(self.declare_parameter('score_fast_weight', 0.75).value)
        self.score_slow_weight = float(self.declare_parameter('score_slow_weight', 0.25).value)
        self.score_rise_tau_sec = float(self.declare_parameter('score_rise_tau_sec', 0.05).value)
        self.score_fall_tau_sec = float(self.declare_parameter('score_fall_tau_sec', 0.50).value)

        # Correction authority:
        # - gain does not go to zero
        # - it drops quickly at spin start
        # - it recovers slowly after spin end
        self.gain_min = float(self.declare_parameter('gain_min', 0.20).value)
        self.gain_down_tau_sec = float(self.declare_parameter('gain_down_tau_sec', 0.06).value)
        self.gain_up_tau_sec = float(self.declare_parameter('gain_up_tau_sec', 1.20).value)

        # Published map_stable->map transform should move gently even if raw TF
        # changes abruptly.
        self.out_yaw_rate_limit_normal = float(
            self.declare_parameter('out_yaw_rate_limit_normal', 0.80).value
        )
        self.out_yaw_rate_limit_spin = float(
            self.declare_parameter('out_yaw_rate_limit_spin', 0.20).value
        )
        self.out_pos_rate_limit_normal = float(
            self.declare_parameter('out_pos_rate_limit_normal', 0.30).value
        )
        self.out_pos_rate_limit_spin = float(
            self.declare_parameter('out_pos_rate_limit_spin', 0.06).value
        )

        # Keep the stable frame from drifting too far away from the raw map.
        self.yaw_lag_cap_normal = float(self.declare_parameter('yaw_lag_cap_normal', 0.40).value)
        self.yaw_lag_cap_spin = float(self.declare_parameter('yaw_lag_cap_spin', 0.20).value)
        self.pos_lag_cap_normal = float(self.declare_parameter('pos_lag_cap_normal', 0.30).value)
        self.pos_lag_cap_spin = float(self.declare_parameter('pos_lag_cap_spin', 0.08).value)

        # When spin relaxes, bleed the stable frame back toward the raw map
        # without releasing all deferred correction at once.
        self.recovery_tau_quiet_sec = float(
            self.declare_parameter('recovery_tau_quiet_sec', 1.20).value
        )
        self.recovery_tau_spin_sec = float(
            self.declare_parameter('recovery_tau_spin_sec', 20.0).value
        )

        self.log_period_sec = float(self.declare_parameter('log_period_sec', 1.0).value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Motion evidence
        self.imu_wz_raw = 0.0
        self.imu_wz_f = 0.0
        self.speed_raw = 0.0
        self.speed_f = 0.0

        # Raw map->odom TF history
        self.have_raw = False
        self.raw_prev = (0.0, 0.0, 0.0)

        # Published map_stable->map TF state
        self.out_pose = (0.0, 0.0, 0.0)

        # Pure-spin state
        self.spin_duration = 0.0
        self.spin_yaw_accum = 0.0
        self.spin_score = 0.0
        self.gain = 1.0

        now = self.get_clock().now()
        self.last_tick = now
        self.last_log_time = now
        self.last_lookup_warn_time = now

        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 50)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.create_timer(1.0 / max(self.publish_hz, 1.0), self._tick)

    def _imu_cb(self, msg: Imu):
        self.imu_wz_raw = msg.angular_velocity.z

    def _odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed_raw = math.hypot(vx, vy)

    def _lookup_raw_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.odom_frame_id,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            now = self.get_clock().now()
            if (now - self.last_lookup_warn_time).nanoseconds >= int(1.0e9):
                self.last_lookup_warn_time = now
                self.get_logger().warning(
                    f'[MapTfStabilizer] waiting for raw TF {self.map_frame_id}->{self.odom_frame_id}: {exc}'
                )
            return None
        return pose_from_transform(transform.transform)

    def _update_spin_state(self, dt):
        self.imu_wz_f = ema(self.imu_wz_f, self.imu_wz_raw, dt, self.wz_filter_tau_sec)
        self.speed_f = ema(self.speed_f, self.speed_raw, dt, self.speed_filter_tau_sec)

        abs_wz = abs(self.imu_wz_f)
        quiet_speed = clamp01(
            (self.spin_speed_quiet - self.speed_f) / max(self.spin_speed_quiet, 1.0e-6)
        )
        wz_term = clamp01(
            (abs_wz - self.spin_wz_start) / max(self.spin_wz_full - self.spin_wz_start, 1.0e-6)
        )
        fast_term = wz_term * quiet_speed

        spin_candidate = abs_wz >= self.spin_wz_start and self.speed_f <= self.spin_speed_quiet
        if spin_candidate:
            self.spin_duration = min(
                self.spin_duration + dt,
                self.spin_duration_ref_sec * 2.0,
            )
            self.spin_yaw_accum = min(
                self.spin_yaw_accum + abs_wz * dt,
                self.spin_yaw_ref_rad * 2.0,
            )
        else:
            duration_decay = (self.spin_duration_ref_sec / max(self.spin_decay_sec, 1.0e-6)) * dt
            yaw_decay = (self.spin_yaw_ref_rad / max(self.spin_decay_sec, 1.0e-6)) * dt
            self.spin_duration = max(0.0, self.spin_duration - duration_decay)
            self.spin_yaw_accum = max(0.0, self.spin_yaw_accum - yaw_decay)

        slow_duration = clamp01(self.spin_duration / max(self.spin_duration_ref_sec, 1.0e-6))
        slow_yaw = clamp01(self.spin_yaw_accum / max(self.spin_yaw_ref_rad, 1.0e-6))
        slow_term = 0.5 * (slow_duration + slow_yaw)

        target_score = clamp01(
            self.score_fast_weight * fast_term + self.score_slow_weight * slow_term
        )
        score_tau = self.score_rise_tau_sec if target_score > self.spin_score else self.score_fall_tau_sec
        self.spin_score = ema(self.spin_score, target_score, dt, score_tau)

        target_gain = 1.0 - self.spin_score * (1.0 - self.gain_min)
        gain_tau = self.gain_down_tau_sec if target_gain < self.gain else self.gain_up_tau_sec
        self.gain = ema(self.gain, target_gain, dt, gain_tau)

    def _update_output_tf(self, raw_now, dt):
        # Raw increment in the raw map frame.
        raw_delta = pose_delta(self.raw_prev, raw_now)

        # Previous effective filtered correction:
        # map_stable->odom_prev = (map_stable->map) * (map->odom_prev)
        filt_prev = pose_compose(self.out_pose, self.raw_prev)

        # Pure spin only reduces the authority of the raw increment.
        scaled_delta = (
            raw_delta[0] * self.gain,
            raw_delta[1] * self.gain,
            raw_delta[2] * self.gain,
        )
        filt_target = pose_compose(filt_prev, scaled_delta)

        # Convert desired filtered correction back to the published upper frame.
        desired_out = pose_compose(filt_target, pose_inverse(raw_now))

        # Bound how far the stable frame is allowed to separate from the raw map.
        score = clamp01(self.spin_score)
        yaw_lag_cap = lerp(self.yaw_lag_cap_normal, self.yaw_lag_cap_spin, score)
        pos_lag_cap = lerp(self.pos_lag_cap_normal, self.pos_lag_cap_spin, score)
        desired_out = (
            *clamp_vector_norm(desired_out[0], desired_out[1], pos_lag_cap),
            clamp(desired_out[2], -yaw_lag_cap, yaw_lag_cap),
        )

        # Do not keep a hidden correction debt. Instead, bleed the current offset
        # back toward identity when spin authority goes away.
        recovery_tau = lerp(self.recovery_tau_quiet_sec, self.recovery_tau_spin_sec, score)
        desired_out = (
            ema(desired_out[0], 0.0, dt, recovery_tau),
            ema(desired_out[1], 0.0, dt, recovery_tau),
            ema(desired_out[2], 0.0, dt, recovery_tau),
        )

        # Final visible TF should never jump in one step.
        yaw_rate_limit = lerp(self.out_yaw_rate_limit_normal, self.out_yaw_rate_limit_spin, score)
        pos_rate_limit = lerp(self.out_pos_rate_limit_normal, self.out_pos_rate_limit_spin, score)
        yaw_step_limit = yaw_rate_limit * dt
        pos_step_limit = pos_rate_limit * dt

        yaw_delta = clamp(
            angle_wrap(desired_out[2] - self.out_pose[2]),
            -yaw_step_limit,
            yaw_step_limit,
        )
        pos_delta_x = desired_out[0] - self.out_pose[0]
        pos_delta_y = desired_out[1] - self.out_pose[1]
        pos_delta_x, pos_delta_y = clamp_vector_norm(pos_delta_x, pos_delta_y, pos_step_limit)

        self.out_pose = (
            self.out_pose[0] + pos_delta_x,
            self.out_pose[1] + pos_delta_y,
            angle_wrap(self.out_pose[2] + yaw_delta),
        )
        self.raw_prev = raw_now

    def _publish_stable_tf(self, now):
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.stable_map_frame_id
        t.child_frame_id = self.map_frame_id
        t.transform.translation.x = self.out_pose[0]
        t.transform.translation.y = self.out_pose[1]
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(self.out_pose[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def _log_state(self, now, raw_now):
        if (now - self.last_log_time).nanoseconds < int(self.log_period_sec * 1.0e9):
            return
        self.last_log_time = now

        raw_yaw = raw_now[2] if raw_now is not None else 0.0
        self.get_logger().info(
            '[MapTfStabilizer] '
            f'imu_wz={self.imu_wz_f:.3f} speed={self.speed_f:.3f} '
            f'score={self.spin_score:.3f} gain={self.gain:.3f} '
            f'spin_t={self.spin_duration:.3f}s spin_yaw={self.spin_yaw_accum:.3f}rad '
            f'raw_yaw={raw_yaw:.3f} stable_map_yaw={self.out_pose[2]:.3f}'
        )

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1.0e-9
        self.last_tick = now
        if dt <= 0.0:
            return

        self._update_spin_state(dt)
        raw_now = self._lookup_raw_tf()

        if raw_now is not None:
            if not self.have_raw:
                self.raw_prev = raw_now
                self.out_pose = (0.0, 0.0, 0.0)
                self.have_raw = True
                self.get_logger().info(
                    '[MapTfStabilizer] first raw map->odom TF received, '
                    'publishing map_stable->map'
                )
            else:
                self._update_output_tf(raw_now, dt)

        # Even if raw TF is temporarily unavailable, keep publishing the last
        # stable transform so the global TF chain does not flap.
        self._publish_stable_tf(now)
        self._log_state(now, raw_now)


def main():
    rclpy.init()
    node = MapTfStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
