#!/usr/bin/env python3
import os
import re
import select
import termios
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


_FLOAT_RE = re.compile(r'^[+-]?\d+(?:\.\d+)?$')
_INT_RE = re.compile(r'^[+-]?\d+$')


@dataclass
class GpsFix:
    latitude: float
    longitude: float
    altitude: float
    hdop: Optional[float]
    fix_status: Optional[int]


def _is_float_token(token: str) -> bool:
    return _FLOAT_RE.match(token) is not None


def _is_int_token(token: str) -> bool:
    return _INT_RE.match(token) is not None


def _safe_float(token: str) -> Optional[float]:
    try:
        return float(token)
    except (TypeError, ValueError):
        return None


def _safe_int(token: str) -> Optional[int]:
    try:
        return int(token)
    except (TypeError, ValueError):
        return None


def _to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return False


def _ddmm_to_decimal(value: float, is_lat: bool) -> float:
    abs_value = abs(value)
    threshold = 90.0 if is_lat else 180.0
    if abs_value <= threshold:
        return value
    degrees = int(abs_value // 100.0)
    minutes = abs_value - degrees * 100.0
    if minutes >= 60.0:
        return value
    dec = degrees + minutes / 60.0
    return -dec if value < 0.0 else dec


def _parse_with_hemisphere(raw: str, hemi: str, is_lat: bool) -> Optional[float]:
    v = _safe_float(raw)
    if v is None:
        return None
    v = _ddmm_to_decimal(v, is_lat=is_lat)
    hemi = hemi.upper()
    if hemi in ('S', 'W'):
        v = -abs(v)
    elif hemi in ('N', 'E'):
        v = abs(v)
    return v


def _termios_baud(baudrate: int):
    table = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }
    return table.get(baudrate, termios.B115200)


class Ec25NavSatBridge(Node):
    def __init__(self) -> None:
        super().__init__('ec25_navsat_bridge')

        self.declare_parameter('device', '/dev/ttyUSB3')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('poll_rate_hz', 2.0)
        self.declare_parameter('read_timeout_sec', 1.2)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('topic_name', '/gps/fix')
        self.declare_parameter('enable_gps_on_start', True)
        self.declare_parameter('qgpsloc_command', 'AT+QGPSLOC=2')
        self.declare_parameter('default_sigma_xy_m', 5.0)
        self.declare_parameter('default_sigma_z_m', 10.0)
        self.declare_parameter('hdop_uere_m', 5.0)
        self.declare_parameter('publish_no_fix', False)

        self.device = str(self.get_parameter('device').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.read_timeout_sec = float(self.get_parameter('read_timeout_sec').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic_name = str(self.get_parameter('topic_name').value)
        self.enable_gps_on_start = _to_bool(self.get_parameter('enable_gps_on_start').value)
        self.qgpsloc_command = str(self.get_parameter('qgpsloc_command').value)
        self.default_sigma_xy_m = float(self.get_parameter('default_sigma_xy_m').value)
        self.default_sigma_z_m = float(self.get_parameter('default_sigma_z_m').value)
        self.hdop_uere_m = float(self.get_parameter('hdop_uere_m').value)
        self.publish_no_fix = _to_bool(self.get_parameter('publish_no_fix').value)

        self.pub = self.create_publisher(NavSatFix, self.topic_name, 10)

        self._fd = -1
        self._io_lock = threading.Lock()
        self._gps_initialized = False
        self._last_warn_time = {}

        timer_period = 1.0 / max(self.poll_rate_hz, 0.2)
        self.timer = self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            f'publishing NavSatFix to {self.topic_name} from {self.device} @ {self.baudrate} bps'
        )

    def destroy_node(self):
        self._close_port()
        super().destroy_node()

    def _warn_throttle(self, key: str, msg: str, interval_sec: float = 5.0) -> None:
        now = time.monotonic()
        last = self._last_warn_time.get(key, 0.0)
        if now - last >= interval_sec:
            self.get_logger().warn(msg)
            self._last_warn_time[key] = now

    def _open_port(self) -> bool:
        if self._fd >= 0:
            return True

        try:
            fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            self._warn_throttle('open_fail', f'cannot open {self.device}: {e}')
            return False

        try:
            attrs = termios.tcgetattr(fd)
            speed = _termios_baud(self.baudrate)

            attrs[0] = termios.IGNPAR
            attrs[1] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[3] = 0
            attrs[4] = speed
            attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0

            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception:
            os.close(fd)
            raise

        self._fd = fd
        self._gps_initialized = False
        self.get_logger().info(f'serial connected: {self.device}')
        return True

    def _close_port(self) -> None:
        if self._fd >= 0:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = -1
            self._gps_initialized = False

    def _flush_input_locked(self) -> None:
        if self._fd < 0:
            return
        while True:
            readable, _, _ = select.select([self._fd], [], [], 0.0)
            if not readable:
                break
            try:
                chunk = os.read(self._fd, 4096)
            except BlockingIOError:
                break
            except OSError:
                break
            if not chunk:
                break

    def _write_line_locked(self, cmd: str) -> None:
        payload = (cmd + '\r').encode('ascii', errors='ignore')
        os.write(self._fd, payload)

    def _read_lines_locked(self, timeout_sec: float) -> List[str]:
        deadline = time.monotonic() + timeout_sec
        buf = b''
        lines: List[str] = []

        while time.monotonic() < deadline:
            wait = max(0.0, min(0.2, deadline - time.monotonic()))
            readable, _, _ = select.select([self._fd], [], [], wait)
            if not readable:
                continue
            try:
                chunk = os.read(self._fd, 4096)
            except BlockingIOError:
                continue
            except OSError:
                break
            if not chunk:
                continue

            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                line = raw.decode(errors='ignore').strip()
                if line:
                    lines.append(line)
                if line == 'OK' or 'ERROR' in line:
                    return lines

        trailing = buf.decode(errors='ignore').strip()
        if trailing:
            lines.append(trailing)
        return lines

    def _query(self, cmd: str, timeout_sec: float) -> List[str]:
        with self._io_lock:
            if self._fd < 0:
                return []
            self._flush_input_locked()
            self._write_line_locked(cmd)
            return self._read_lines_locked(timeout_sec)

    def _initialize_gps(self) -> None:
        if self._gps_initialized:
            return

        at_rsp = self._query('AT', 0.6)
        if not any(line == 'OK' for line in at_rsp):
            self._warn_throttle('at_no_ok', 'EC25 AT handshake failed')
            return

        qgps_rsp = self._query('AT+QGPS?', 0.8)
        gps_on = any('+QGPS: 1' in line for line in qgps_rsp)

        if not gps_on and self.enable_gps_on_start:
            _ = self._query('AT+QGPS=1', 1.0)
            time.sleep(0.2)

        self._gps_initialized = True

    def _find_qgpsloc_payload(self, lines: List[str]) -> Optional[str]:
        for line in lines:
            if '+QGPSLOC:' in line:
                return line.split(':', 1)[1].strip()
        return None

    def _parse_qgpsloc(self, payload: str) -> Optional[GpsFix]:
        tokens = [tok.strip() for tok in payload.split(',')]
        if len(tokens) < 2:
            return None

        lat = None
        lon = None
        hdop = None
        alt = float('nan')
        fix_status = None

        idx_ns = next((i for i, t in enumerate(tokens) if t.upper() in ('N', 'S')), None)
        idx_ew = next((i for i, t in enumerate(tokens) if t.upper() in ('E', 'W')), None)

        if idx_ns is not None and idx_ew is not None and idx_ns > 0 and idx_ew > 0:
            lat = _parse_with_hemisphere(tokens[idx_ns - 1], tokens[idx_ns], is_lat=True)
            lon = _parse_with_hemisphere(tokens[idx_ew - 1], tokens[idx_ew], is_lat=False)

            if idx_ew + 1 < len(tokens):
                hdop = _safe_float(tokens[idx_ew + 1])
            if idx_ew + 2 < len(tokens):
                alt_v = _safe_float(tokens[idx_ew + 2])
                if alt_v is not None:
                    alt = alt_v
            if idx_ew + 3 < len(tokens) and _is_int_token(tokens[idx_ew + 3]):
                fix_status = _safe_int(tokens[idx_ew + 3])
        else:
            lat_idx = -1
            for i in range(0, len(tokens) - 1):
                if not (_is_float_token(tokens[i]) and _is_float_token(tokens[i + 1])):
                    continue
                a = float(tokens[i])
                b = float(tokens[i + 1])
                if abs(a) <= 90.0 and abs(b) <= 180.0:
                    lat = a
                    lon = b
                    lat_idx = i
                    break
                if abs(a) <= 180.0 and abs(b) <= 90.0:
                    lat = b
                    lon = a
                    lat_idx = i
                    break

            if lat_idx >= 0:
                if lat_idx + 2 < len(tokens):
                    hdop = _safe_float(tokens[lat_idx + 2])
                if lat_idx + 3 < len(tokens):
                    alt_v = _safe_float(tokens[lat_idx + 3])
                    if alt_v is not None:
                        alt = alt_v
                if lat_idx + 4 < len(tokens) and _is_int_token(tokens[lat_idx + 4]):
                    fix_status = _safe_int(tokens[lat_idx + 4])

        if lat is None or lon is None:
            return None

        if abs(lat) > 90.0 or abs(lon) > 180.0:
            return None

        return GpsFix(latitude=lat, longitude=lon, altitude=alt, hdop=hdop, fix_status=fix_status)

    def _poll_fix(self) -> Optional[GpsFix]:
        lines = self._query(self.qgpsloc_command, self.read_timeout_sec)
        if not lines:
            self._warn_throttle('empty_rsp', 'no response from modem')
            return None

        for line in lines:
            if '+CME ERROR:' in line:
                code = line.split(':', 1)[1].strip()
                # 516: no fix yet (common indoors), 502: operation not allowed
                if code not in ('516', '502'):
                    self._warn_throttle('cme', f'modem error {line}')
                return None

        payload = self._find_qgpsloc_payload(lines)
        if payload is None:
            self._warn_throttle('no_qgpsloc', f'QGPSLOC not found in response: {lines}')
            return None

        fix = self._parse_qgpsloc(payload)
        if fix is None:
            self._warn_throttle('parse_fail', f'failed to parse QGPSLOC payload: {payload}')
            return None

        return fix

    def _publish_fix(self, fix: GpsFix) -> None:
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        fix_ok = True
        if fix.fix_status is not None:
            fix_ok = fix.fix_status > 0

        if fix_ok:
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = fix.latitude
        msg.longitude = fix.longitude
        msg.altitude = fix.altitude

        sigma_xy = self.default_sigma_xy_m
        if fix.hdop is not None and fix.hdop > 0.0:
            sigma_xy = max(self.default_sigma_xy_m, fix.hdop * self.hdop_uere_m)
        sigma_z = self.default_sigma_z_m

        var_xy = sigma_xy * sigma_xy
        var_z = sigma_z * sigma_z
        msg.position_covariance = [
            var_xy, 0.0, 0.0,
            0.0, var_xy, 0.0,
            0.0, 0.0, var_z,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        if fix_ok or self.publish_no_fix:
            self.pub.publish(msg)

    def _on_timer(self) -> None:
        if not self._open_port():
            return

        try:
            self._initialize_gps()
            fix = self._poll_fix()
            if fix is None:
                return
            self._publish_fix(fix)
        except Exception as e:  # noqa: BLE001
            self._warn_throttle('poll_exception', f'gps poll error: {e}')
            self._close_port()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ec25NavSatBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
