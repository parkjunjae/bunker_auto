#!/usr/bin/env python3
import csv
import math
import os
import time
from datetime import datetime
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from bunker_msgs.msg import BunkerStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class BunkerMotorProbe(Node):
    def __init__(self):
        super().__init__('bunker_motor_probe')
        self.sub = self.create_subscription(BunkerStatus, '/bunker_status', self.cb, 20)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.seg_sub = self.create_subscription(String, '/probe_segment', self._cb_segment_cmd, 10)

        self.prev = {}  # motor_id -> (t, pulse_count)
        self.last_print = time.monotonic()
        self.stats = defaultdict(dict)

        # watchdog
        self.start_time = time.monotonic()
        self.last_msg_time = None
        self.status_timeout_sec = 2.0
        self.no_msg_grace_sec = 5.0
        self.stop_on_timeout = True
        self._last_watchdog_log = 0.0

        # averages
        self.window_sec = 10.0
        self.window = defaultdict(lambda: deque())  # (t, rpm, current, dpps, cpr_est)
        self.acc = defaultdict(lambda: {
            'n': 0,
            'rpm_sum': 0.0,
            'cur_sum': 0.0,
            'dpps_sum': 0.0,
            'cpr_sum': 0.0,
            'dpps_n': 0,
            'cpr_n': 0,
        })

        # segment
        self.stop_v_thresh = 0.03
        self.stop_w_thresh = 0.05
        self.straight_v_thresh = 0.08
        self.straight_w_thresh = 0.10
        self.rotate_w_thresh = 0.20
        self.rotate_v_thresh = 0.05
        self.min_segment_sec = 20.0

        self.segment = 'other'
        self.segment_override = None  # None=auto
        self.segment_switch_time = time.monotonic()

        self.window_seg = defaultdict(lambda: deque())  # (t, rpm, cur, dpps, cpr)
        self.acc_seg = defaultdict(lambda: {
            'n': 0,
            'rpm_sum': 0.0,
            'cur_sum': 0.0,
            'dpps_sum': 0.0,
            'cpr_sum': 0.0,
            'dpps_n': 0,
            'cpr_n': 0,
            't_first': None,
            't_last': None,
        })

        self._setup_csv()
        self.watchdog = self.create_timer(0.5, self._watchdog)

        self.get_logger().info('subscribed: /bunker_status')
        self.get_logger().info('manual segment cmd: ros2 topic pub /probe_segment std_msgs/msg/String "{data: straight}" -1')
        self.get_logger().info('segment options: straight | rotate_in_place | stop | other | auto')
        self.get_logger().info(f'csv raw: {self.csv_path_raw}')
        self.get_logger().info(f'csv seg: {self.csv_path_seg}')

    def _setup_csv(self):
        log_dir = '/home/atoz/ca_ws/logs'
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path_raw = os.path.join(log_dir, f'bunker_motor_raw_{ts}.csv')
        self.csv_path_seg = os.path.join(log_dir, f'bunker_segment_avg_{ts}.csv')
        self.csv_raw_f = open(self.csv_path_raw, 'w', newline='')
        self.csv_seg_f = open(self.csv_path_seg, 'w', newline='')
        self.csv_raw_w = csv.writer(self.csv_raw_f)
        self.csv_seg_w = csv.writer(self.csv_seg_f)
        self.csv_raw_w.writerow([
            't', 'segment', 'v', 'w', 'motor_id', 'rpm', 'current', 'pulse_count', 'dpps', 'cpr_est'
        ])
        self.csv_seg_w.writerow([
            't', 'segment', 'window_sec', 'rpm_avg_10s', 'cur_avg_10s', 'dpps_avg_10s',
            'cpr_avg_10s', 'rpm_avg_all', 'cur_avg_all', 'dpps_avg_all', 'cpr_avg_all',
            'n_all', 'duration_sec', 'data_ok'
        ])

    def _cb_segment_cmd(self, msg: String):
        val = msg.data.strip().lower()
        allowed = {'straight', 'rotate_in_place', 'stop', 'other', 'auto'}
        if val not in allowed:
            self.get_logger().warn(f"invalid segment cmd '{val}', allowed={sorted(allowed)}")
            return
        if val == 'auto':
            self.segment_override = None
            self.get_logger().info('segment mode -> AUTO')
        else:
            self.segment_override = val
            self.get_logger().info(f'segment mode -> MANUAL ({val})')

    def _classify_segment(self, v, w):
        av = abs(v)
        aw = abs(w)
        if av < self.stop_v_thresh and aw < self.stop_w_thresh:
            return 'stop'
        if av > self.straight_v_thresh and aw < self.straight_w_thresh:
            return 'straight'
        if aw > self.rotate_w_thresh and av < self.rotate_v_thresh:
            return 'rotate_in_place'
        return 'other'

    def cb(self, msg: BunkerStatus):
        now = time.monotonic()
        self.last_msg_time = now

        lv = float(msg.linear_velocity)
        av = float(msg.angular_velocity)

        auto_seg = self._classify_segment(lv, av)
        new_seg = self.segment_override if self.segment_override is not None else auto_seg
        if new_seg != self.segment:
            self.segment = new_seg
            self.segment_switch_time = now
            print(f"\n===== SEGMENT SWITCH -> {self.segment} =====")

        rpm_now_list = []
        cur_now_list = []
        dpps_now_list = []
        cpr_now_list = []

        for s in msg.actuator_states:
            mid = int(s.motor_id)
            rpm = float(s.rpm)
            pulse = int(s.pulse_count)
            current = float(s.current)

            dpps = float('nan')
            cpr_est = float('nan')

            if mid in self.prev:
                t0, p0 = self.prev[mid]
                dt = now - t0
                dp = pulse - p0
                if dt > 1e-3:
                    dpps = dp / dt
                    if abs(rpm) > 3.0:
                        cpr_est = abs(dpps) * 60.0 / abs(rpm)

            self.prev[mid] = (now, pulse)
            self.stats[mid] = {
                'rpm': rpm,
                'current': current,
                'pulse': pulse,
                'dpps': dpps,
                'cpr_est': cpr_est,
            }

            self.csv_raw_w.writerow([
                time.time(), self.segment, lv, av, mid, rpm, current, pulse, dpps, cpr_est
            ])

            self.window[mid].append((now, rpm, current, dpps, cpr_est))
            while self.window[mid] and (now - self.window[mid][0][0]) > self.window_sec:
                self.window[mid].popleft()

            a = self.acc[mid]
            a['n'] += 1
            a['rpm_sum'] += rpm
            a['cur_sum'] += current
            if not math.isnan(dpps):
                a['dpps_sum'] += dpps
                a['dpps_n'] += 1
            if not math.isnan(cpr_est):
                a['cpr_sum'] += cpr_est
                a['cpr_n'] += 1

            rpm_now_list.append(rpm)
            cur_now_list.append(current)
            if not math.isnan(dpps):
                dpps_now_list.append(dpps)
            if not math.isnan(cpr_est):
                cpr_now_list.append(cpr_est)

        # segment-level robot mean per callback
        if rpm_now_list:
            rpm_now = sum(rpm_now_list) / len(rpm_now_list)
            cur_now = sum(cur_now_list) / len(cur_now_list)
            dpps_now = sum(dpps_now_list) / len(dpps_now_list) if dpps_now_list else float('nan')
            cpr_now = sum(cpr_now_list) / len(cpr_now_list) if cpr_now_list else float('nan')

            self.window_seg[self.segment].append((now, rpm_now, cur_now, dpps_now, cpr_now))
            while self.window_seg[self.segment] and (now - self.window_seg[self.segment][0][0]) > self.window_sec:
                self.window_seg[self.segment].popleft()

            sa = self.acc_seg[self.segment]
            sa['n'] += 1
            sa['rpm_sum'] += rpm_now
            sa['cur_sum'] += cur_now
            if sa['t_first'] is None:
                sa['t_first'] = now
            sa['t_last'] = now
            if not math.isnan(dpps_now):
                sa['dpps_sum'] += dpps_now
                sa['dpps_n'] += 1
            if not math.isnan(cpr_now):
                sa['cpr_sum'] += cpr_now
                sa['cpr_n'] += 1

        if now - self.last_print >= 1.0:
            self.last_print = now
            mode = 'MANUAL' if self.segment_override is not None else 'AUTO'
            seg_elapsed = now - self.segment_switch_time
            print('\n=== bunker_status ===')
            print(f'segment={self.segment} ({mode}, +{seg_elapsed:.1f}s), v={lv:+.3f} m/s, w={av:+.3f} rad/s')

            robot_rpm_w, robot_cur_w, robot_dpps_w, robot_cpr_w = [], [], [], []
            robot_rpm_all, robot_cur_all, robot_dpps_all, robot_cpr_all = [], [], [], []

            for mid in sorted(self.stats.keys()):
                s = self.stats[mid]
                w = self.window[mid]
                if w:
                    rpm_w = sum(x[1] for x in w) / len(w)
                    cur_w = sum(x[2] for x in w) / len(w)
                    dpps_vals = [x[3] for x in w if not math.isnan(x[3])]
                    cpr_vals = [x[4] for x in w if not math.isnan(x[4])]
                    dpps_w = sum(dpps_vals) / len(dpps_vals) if dpps_vals else float('nan')
                    cpr_w = sum(cpr_vals) / len(cpr_vals) if cpr_vals else float('nan')
                else:
                    rpm_w = cur_w = dpps_w = cpr_w = float('nan')

                a = self.acc[mid]
                rpm_all = a['rpm_sum'] / a['n'] if a['n'] else float('nan')
                cur_all = a['cur_sum'] / a['n'] if a['n'] else float('nan')
                dpps_all = a['dpps_sum'] / a['dpps_n'] if a['dpps_n'] else float('nan')
                cpr_all = a['cpr_sum'] / a['cpr_n'] if a['cpr_n'] else float('nan')

                if not math.isnan(rpm_w):
                    robot_rpm_w.append(rpm_w)
                if not math.isnan(cur_w):
                    robot_cur_w.append(cur_w)
                if not math.isnan(dpps_w):
                    robot_dpps_w.append(dpps_w)
                if not math.isnan(cpr_w):
                    robot_cpr_w.append(cpr_w)

                if not math.isnan(rpm_all):
                    robot_rpm_all.append(rpm_all)
                if not math.isnan(cur_all):
                    robot_cur_all.append(cur_all)
                if not math.isnan(dpps_all):
                    robot_dpps_all.append(dpps_all)
                if not math.isnan(cpr_all):
                    robot_cpr_all.append(cpr_all)

                dpps_txt = 'nan' if math.isnan(s['dpps']) else f"{s['dpps']:+.1f}"
                cpr_txt = 'nan' if math.isnan(s['cpr_est']) else f"{s['cpr_est']:.1f}"
                dpps_w_txt = 'nan' if math.isnan(dpps_w) else f"{dpps_w:+.1f}"
                cpr_w_txt = 'nan' if math.isnan(cpr_w) else f"{cpr_w:.1f}"
                dpps_all_txt = 'nan' if math.isnan(dpps_all) else f"{dpps_all:+.1f}"
                cpr_all_txt = 'nan' if math.isnan(cpr_all) else f"{cpr_all:.1f}"

                print(f"motor {mid}: rpm={s['rpm']:+7.1f}, I={s['current']:+5.2f}A, pulse={s['pulse']:>10d}, dp/s={dpps_txt:>8}, cpr_est={cpr_txt}")
                print(f"  avg(10s): rpm={rpm_w:+7.1f}, I={cur_w:+5.2f}A, dp/s={dpps_w_txt:>8}, cpr={cpr_w_txt}")
                print(f"  avg(all): rpm={rpm_all:+7.1f}, I={cur_all:+5.2f}A, dp/s={dpps_all_txt:>8}, cpr={cpr_all_txt}, n={a['n']}")

            if robot_rpm_w:
                print(
                    f"ROBOT avg(10s): rpm={sum(robot_rpm_w)/len(robot_rpm_w):+7.1f}, "
                    f"I={sum(robot_cur_w)/len(robot_cur_w):+5.2f}A, "
                    f"dp/s={sum(robot_dpps_w)/len(robot_dpps_w):+8.1f}, "
                    f"cpr={sum(robot_cpr_w)/len(robot_cpr_w):.1f}"
                )
            if robot_rpm_all:
                print(
                    f"ROBOT avg(all): rpm={sum(robot_rpm_all)/len(robot_rpm_all):+7.1f}, "
                    f"I={sum(robot_cur_all)/len(robot_cur_all):+5.2f}A, "
                    f"dp/s={sum(robot_dpps_all)/len(robot_dpps_all):+8.1f}, "
                    f"cpr={sum(robot_cpr_all)/len(robot_cpr_all):.1f}"
                )

            print('--- SEGMENT AVG ---')
            for seg in ('straight', 'rotate_in_place', 'stop', 'other'):
                wseg = self.window_seg[seg]
                aseg = self.acc_seg[seg]
                if not aseg['n']:
                    continue

                if wseg:
                    rpm_w = sum(x[1] for x in wseg) / len(wseg)
                    cur_w = sum(x[2] for x in wseg) / len(wseg)
                    dpps_vals = [x[3] for x in wseg if not math.isnan(x[3])]
                    cpr_vals = [x[4] for x in wseg if not math.isnan(x[4])]
                    dpps_w = sum(dpps_vals) / len(dpps_vals) if dpps_vals else float('nan')
                    cpr_w = sum(cpr_vals) / len(cpr_vals) if cpr_vals else float('nan')
                else:
                    rpm_w = cur_w = dpps_w = cpr_w = float('nan')

                rpm_all = aseg['rpm_sum'] / aseg['n']
                cur_all = aseg['cur_sum'] / aseg['n']
                dpps_all = aseg['dpps_sum'] / aseg['dpps_n'] if aseg['dpps_n'] else float('nan')
                cpr_all = aseg['cpr_sum'] / aseg['cpr_n'] if aseg['cpr_n'] else float('nan')
                dur = (aseg['t_last'] - aseg['t_first']) if (aseg['t_first'] is not None and aseg['t_last'] is not None) else 0.0
                data_ok = dur >= self.min_segment_sec

                dpps_w_txt = 'nan' if math.isnan(dpps_w) else f"{dpps_w:+.1f}"
                cpr_w_txt = 'nan' if math.isnan(cpr_w) else f"{cpr_w:.1f}"
                dpps_all_txt = 'nan' if math.isnan(dpps_all) else f"{dpps_all:+.1f}"
                cpr_all_txt = 'nan' if math.isnan(cpr_all) else f"{cpr_all:.1f}"

                print(
                    f"{seg:>15}: avg(10s) rpm={rpm_w:+7.1f}, I={cur_w:+5.2f}A, "
                    f"dp/s={dpps_w_txt:>8}, cpr={cpr_w_txt} | "
                    f"avg(all) rpm={rpm_all:+7.1f}, I={cur_all:+5.2f}A, "
                    f"dp/s={dpps_all_txt:>8}, cpr={cpr_all_txt}, n={aseg['n']}, dur={dur:.1f}s"
                )
                if not data_ok:
                    print(f"  -> LOW_DATA: {self.min_segment_sec:.0f}s 이상 수집 권장")

                self.csv_seg_w.writerow([
                    time.time(), seg, self.window_sec, rpm_w, cur_w, dpps_w, cpr_w,
                    rpm_all, cur_all, dpps_all, cpr_all, aseg['n'], dur, int(data_ok)
                ])

            self.csv_raw_f.flush()
            self.csv_seg_f.flush()

    def _publish_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def _watchdog(self):
        now = time.monotonic()
        if self.last_msg_time is None:
            if (now - self.start_time) > self.no_msg_grace_sec:
                if (now - self._last_watchdog_log) > 2.0:
                    self._last_watchdog_log = now
                    self.get_logger().error('No /bunker_status received yet. Check bunker_base launch/CAN.')
                    if self.stop_on_timeout:
                        self._publish_stop()
            return

        if (now - self.last_msg_time) > self.status_timeout_sec:
            if (now - self._last_watchdog_log) > 1.0:
                self._last_watchdog_log = now
                self.get_logger().error(f'/bunker_status timeout: {(now - self.last_msg_time):.2f}s')
                if self.stop_on_timeout:
                    self.get_logger().warn('Publishing safe stop cmd_vel=0 due to timeout.')
                    self._publish_stop()


def main():
    rclpy.init()
    node = BunkerMotorProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_raw_f.close()
        node.csv_seg_f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
