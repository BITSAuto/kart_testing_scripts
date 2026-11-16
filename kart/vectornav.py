"""VectorNav input: none, direct serial (ASCII async output), or ROS 2 topics.

All sources fill the same state dict:
    t          monotonic time of the last update
    speed_mps  horizontal speed
    x_m, y_m   local east/north position from the first fix (m)
    yaw_deg    heading as reported (VectorNav: 0 = north, clockwise)
    yaw_rate   rad/s, positive = turning LEFT
    lat, lon, fix
"""
import math
import threading
import time

R_EARTH = 6378137.0


def _checksum_ok(line):
    if "*" not in line:
        return True
    body, cs = line[1:].split("*", 1)
    try:
        want = int(cs[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body.encode():
        got ^= ch
    return got == want


class _Base:
    name = "none"

    def __init__(self, cfg, rec=None):
        self.cfg, self.rec = cfg, rec
        self.state = {}
        self.count = 0
        self.history = []     # (t, speed_mps, x_m, y_m, yaw_deg, yaw_rate)
        self.stale_s = cfg["vectornav"].get("stale_s", 1.0)
        self._origin = None
        self._stop = threading.Event()

    def start(self):
        pass

    def close(self):
        self._stop.set()

    def available(self):
        t = self.state.get("t")
        return t is not None and time.monotonic() - t < self.stale_s

    def speed_kmh(self):
        v = self.state.get("speed_mps")
        return None if v is None or not self.available() else v * 3.6

    def _latlon(self, lat, lon):
        if self._origin is None:
            self._origin = (lat, lon)
        lat0, lon0 = self._origin
        x = math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
        y = math.radians(lat - lat0) * R_EARTH
        return x, y

    def _update(self, **kw):
        t = time.monotonic()
        self.state.update(kw, t=t, source=self.name)
        self.count += 1
        s = self.state
        self.history.append((t, s.get("speed_mps"), s.get("x_m"), s.get("y_m"),
                             s.get("yaw_deg"), s.get("yaw_rate")))
        if self.rec:
            self.rec.vn(t, self.state)


class NoVectorNav(_Base):
    pass


class SerialVectorNav(_Base):
    """Parses $VNINS (position, velocity, heading) and $VNYMR (gyro for yaw rate)."""
    name = "serial"

    def start(self):
        import serial
        v = self.cfg["vectornav"]
        self.ser = serial.Serial(v["port"], int(v["baud"]), timeout=0.05)
        threading.Thread(target=self._run, daemon=True).start()

    def close(self):
        super().close()
        try:
            self.ser.close()
        except Exception:
            pass

    def _run(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.parse(line.decode(errors="replace").strip())

    def parse(self, line):
        if not line.startswith("$VN") or not _checksum_ok(line):
            return
        f = line.split("*")[0].split(",")
        try:
            if f[0] == "$VNINS" and len(f) >= 13:
                yaw, lat, lon = float(f[4]), float(f[7]), float(f[8])
                vn, ve = float(f[10]), float(f[11])
                status = int(f[3], 16)
                x, y = self._latlon(lat, lon)
                self._update(yaw_deg=yaw, lat=lat, lon=lon, x_m=x, y_m=y,
                             speed_mps=math.hypot(vn, ve), fix=(status >> 2) & 1,
                             ins_mode=status & 3)
            elif f[0] == "$VNYMR" and len(f) >= 13:
                # gyro is in the body frame (x fwd, y right, z down): left turn = -gz
                self._update(yaw_deg=float(f[1]), yaw_rate=-float(f[12]))
        except (ValueError, IndexError):
            pass


class Ros2VectorNav(_Base):
    name = "ros2"

    def start(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        r = self.cfg["vectornav"]["ros2"]
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("kart_test_vectornav")
        sub = self.node.create_subscription
        if r.get("odom_topic"):
            from nav_msgs.msg import Odometry
            sub(Odometry, r["odom_topic"], self._odom, qos_profile_sensor_data)
        else:
            from geometry_msgs.msg import TwistWithCovarianceStamped
            from sensor_msgs.msg import NavSatFix
            if r.get("fix_topic"):
                sub(NavSatFix, r["fix_topic"], self._fix, qos_profile_sensor_data)
            if r.get("velocity_topic"):
                sub(TwistWithCovarianceStamped, r["velocity_topic"], self._vel,
                    qos_profile_sensor_data)
        if r.get("imu_topic"):
            from sensor_msgs.msg import Imu
            sub(Imu, r["imu_topic"], self._imu, qos_profile_sensor_data)
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        import rclpy
        while not self._stop.is_set() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def close(self):
        super().close()
        try:
            import rclpy
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def _odom(self, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self._update(x_m=p.x, y_m=p.y, yaw_deg=yaw, speed_mps=math.hypot(v.x, v.y),
                     yaw_rate=m.twist.twist.angular.z, fix=1)

    def _fix(self, m):
        x, y = self._latlon(m.latitude, m.longitude)
        self._update(lat=m.latitude, lon=m.longitude, x_m=x, y_m=y, fix=int(m.status.status >= 0))

    def _vel(self, m):
        v = m.twist.twist.linear
        self._update(speed_mps=math.hypot(v.x, v.y))

    def _imu(self, m):
        self._update(yaw_rate=m.angular_velocity.z)


def make_vectornav(cfg, rec=None):
    src = cfg["vectornav"].get("source", "none")
    cls = {"serial": SerialVectorNav, "ros2": Ros2VectorNav}.get(src, NoVectorNav)
    return cls(cfg, rec)
