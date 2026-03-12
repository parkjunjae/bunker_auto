#!/usr/bin/env python3
"""
map_topic_stabilizer.py

Pure spin 구간에서는 raw /rtabmap/map redraw가 consumer에 바로 전파되지 않도록
last-good occupancy grid를 유지하는 1차 relay 노드다.

구조:
1. input  : /rtabmap/map          (raw occupancy grid, frame_id=map)
2. output : /rtabmap/map_stable   (filtered occupancy grid, frame_id=map_stable)

설계 원칙:
1. 일반 주행에서는 raw map을 그대로 통과시킨다.
2. pure spin 동안에는 마지막으로 publish한 stable map을 유지한다.
3. spin이 끝났다고 바로 raw map으로 snap-back 하지 않는다.
4. translation evidence가 다시 생긴 뒤에만 refresh를 허용한다.
5. 첫 stable map은 반드시 빨리 publish해서 startup 공백을 만들지 않는다.

주의:
- 이 노드는 root cause를 고치는 source-level fix가 아니다.
- 현재 단계에서는 raw /rtabmap/map redraw가 consumer에 바로 전파되는 경로를
  차단하는 보완층이다.
- 1차 구현은 full OccupancyGrid만 relay하며 map_updates는 다루지 않는다.
"""

import copy
import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def clamp01(value):
    return clamp(value, 0.0, 1.0)


def ema(current, target, dt, tau):
    if tau <= 1.0e-6:
        return target
    alpha = 1.0 - math.exp(-dt / tau)
    return current + alpha * (target - current)


class MapTopicStabilizer(Node):
    def __init__(self):
        super().__init__('map_topic_stabilizer')

        self.input_map_topic = self.declare_parameter('input_map_topic', '/rtabmap/map').value
        self.output_map_topic = self.declare_parameter('output_map_topic', '/rtabmap/map_stable').value
        self.stable_map_frame_id = self.declare_parameter('stable_map_frame_id', 'map_stable').value
        self.imu_topic = self.declare_parameter('imu_topic', '/camera/camera/imu_fixed').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odometry/filtered').value
        self.tick_hz = float(self.declare_parameter('tick_hz', 20.0).value)

        # Keep detector behavior aligned with the TF stabilizer so both layers
        # react to the same pure spin window.
        self.wz_filter_tau_sec = float(self.declare_parameter('wz_filter_tau_sec', 0.06).value)
        self.speed_filter_tau_sec = float(self.declare_parameter('speed_filter_tau_sec', 0.12).value)
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

        # Hold/refresh policy:
        # - enter hold when pure spin score is confidently high
        # - leave hold only after spin relaxes and translation evidence returns
        # - delay refresh slightly to avoid immediate raw-map snap-back
        self.hold_enter_score = float(self.declare_parameter('hold_enter_score', 0.55).value)
        self.hold_exit_score = float(self.declare_parameter('hold_exit_score', 0.20).value)
        self.translation_release_speed = float(
            self.declare_parameter('translation_release_speed', 0.06).value
        )
        self.translation_release_hold_sec = float(
            self.declare_parameter('translation_release_hold_sec', 0.30).value
        )
        self.refresh_settle_sec = float(
            self.declare_parameter('refresh_settle_sec', 0.25).value
        )
        self.max_hold_sec = float(self.declare_parameter('max_hold_sec', 4.0).value)
        self.log_period_sec = float(self.declare_parameter('log_period_sec', 1.0).value)

        qos_map = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_pub = self.create_publisher(OccupancyGrid, self.output_map_topic, qos_map)
        self.create_subscription(OccupancyGrid, self.input_map_topic, self._map_cb, qos_map)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 50)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)

        self.imu_wz_raw = 0.0
        self.imu_wz_f = 0.0
        self.speed_raw = 0.0
        self.speed_f = 0.0
        self.spin_duration = 0.0
        self.spin_yaw_accum = 0.0
        self.spin_score = 0.0

        self.latest_raw_map = None
        self.last_published_map = None
        self.hold_active = False
        self.hold_duration = 0.0
        self.translation_release_duration = 0.0
        self.pending_refresh = False
        self.pending_refresh_duration = 0.0

        now = self.get_clock().now()
        self.last_tick = now
        self.last_log_time = now

        self.create_timer(1.0 / max(self.tick_hz, 1.0), self._tick)

    def _imu_cb(self, msg: Imu):
        self.imu_wz_raw = msg.angular_velocity.z

    def _odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed_raw = math.hypot(vx, vy)

    def _map_cb(self, msg: OccupancyGrid):
        self.latest_raw_map = msg

        # Startup 공백 방지: stable map이 아직 없으면 첫 raw map을 즉시 publish.
        if self.last_published_map is None:
            self._publish_stable_map(msg, 'initial')
            return

        # 일반 구간에서는 raw map을 그대로 소비자에게 전달.
        if not self.hold_active and not self.pending_refresh:
            self._publish_stable_map(msg, 'pass_through')

    def _publish_stable_map(self, raw_msg: OccupancyGrid, reason: str):
        stable_msg = copy.deepcopy(raw_msg)
        stable_msg.header.frame_id = self.stable_map_frame_id
        self.map_pub.publish(stable_msg)
        self.last_published_map = stable_msg
        self.get_logger().info(
            f'[MapTopicStabilizer] publish reason={reason} frame={stable_msg.header.frame_id} '
            f'size={stable_msg.info.width}x{stable_msg.info.height}'
        )

    def _update_spin_score(self, dt: float):
        self.imu_wz_f = ema(self.imu_wz_f, self.imu_wz_raw, dt, self.wz_filter_tau_sec)
        self.speed_f = ema(self.speed_f, self.speed_raw, dt, self.speed_filter_tau_sec)

        abs_wz = abs(self.imu_wz_f)
        wz_term = clamp01(
            (abs_wz - self.spin_wz_start) / max(self.spin_wz_full - self.spin_wz_start, 1.0e-6)
        )
        quiet_term = 1.0 - clamp01(self.speed_f / max(self.spin_speed_quiet, 1.0e-6))
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

        score_target = clamp01(
            self.score_fast_weight * fast_term + self.score_slow_weight * slow_term
        )
        tau = self.score_rise_tau_sec if score_target >= self.spin_score else self.score_fall_tau_sec
        self.spin_score = ema(self.spin_score, score_target, dt, tau)

    def _tick(self):
        now = self.get_clock().now()
        dt = max((now - self.last_tick).nanoseconds / 1.0e9, 1.0e-3)
        self.last_tick = now

        self._update_spin_score(dt)

        if self.hold_active:
            self.hold_duration += dt

            if (
                self.spin_score <= self.hold_exit_score
                and self.speed_f >= self.translation_release_speed
            ):
                self.translation_release_duration += dt
            else:
                self.translation_release_duration = 0.0

            timed_out = self.hold_duration >= self.max_hold_sec
            released = self.translation_release_duration >= self.translation_release_hold_sec
            if timed_out or released:
                self.hold_active = False
                self.pending_refresh = True
                self.pending_refresh_duration = 0.0
                release_reason = 'timeout' if timed_out else 'translation_reacquired'
                self.get_logger().warning(
                    f'[MapTopicStabilizer] hold_exit reason={release_reason} '
                    f'score={self.spin_score:.3f} wz={self.imu_wz_f:.3f} speed={self.speed_f:.3f}'
                )
        else:
            if self.spin_score >= self.hold_enter_score and self.last_published_map is not None:
                self.hold_active = True
                self.hold_duration = 0.0
                self.translation_release_duration = 0.0
                self.pending_refresh = False
                self.pending_refresh_duration = 0.0
                self.get_logger().warning(
                    f'[MapTopicStabilizer] hold_enter score={self.spin_score:.3f} '
                    f'wz={self.imu_wz_f:.3f} speed={self.speed_f:.3f}'
                )

        if self.pending_refresh:
            self.pending_refresh_duration += dt
            if (
                self.latest_raw_map is not None
                and self.pending_refresh_duration >= self.refresh_settle_sec
                and self.speed_f >= self.translation_release_speed
                and self.spin_score <= self.hold_exit_score
            ):
                self.pending_refresh = False
                self.pending_refresh_duration = 0.0
                self._publish_stable_map(self.latest_raw_map, 'refresh_after_spin')

        if (now - self.last_log_time).nanoseconds >= int(self.log_period_sec * 1.0e9):
            self.last_log_time = now
            self.get_logger().info(
                f'[MapTopicStabilizer] hold={int(self.hold_active)} pending={int(self.pending_refresh)} '
                f'score={self.spin_score:.3f} wz={self.imu_wz_f:.3f} speed={self.speed_f:.3f} '
                f'hold_t={self.hold_duration:.2f} rel_t={self.translation_release_duration:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MapTopicStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
