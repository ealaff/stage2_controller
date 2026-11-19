import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# Settings
WAYPOINTS = [(9.80, 8.90), (-13.60, 8.95), (-13.65, 0.25)]
WAYPOINT_RADIUS = 0.80
MAX_SPEED = 6.0
BOTTOM_SPEED = 2.40
CHICANE_SPEED = 1.85
CLOSE_GAP_SPEED = 1.35
CREEP_SPEED = 0.45
ACCEL_LIMIT = 3.5
BRAKING_ACCEL = 4.0
LATERAL_ACCEL = 3.2
MAX_STEER = 0.4
LOOKAHEAD_MIN, LOOKAHEAD_MAX = 0.80, 1.45
GAP_CLEARANCE = 0.48
BUBBLE_RADIUS = 0.21
SENSOR_TIMEOUT = 0.35
CONTROL_PERIOD = 0.025
BRAKE_TIME, REVERSE_TIME, REVERSE_SPEED = 0.12, 0.90, -0.45
STUCK_TIMEOUT = 0.9
COLLISION_DEBOUNCE = 0.9
WHEELBASE = 0.3302
LIDAR_OFFSET = 0.275


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class Stage2Controller(Node):

    def __init__(self):
        super().__init__('stage2_controller')

        self.lookahead = LOOKAHEAD_MIN
        self.waypoint_index = 0
        self.goal_weight = 1.35
        self.bottom_segment = False
        self.creep_only = False

        self.pose = self.scan = None
        self.last_odom = self.last_scan = None
        self.actual_speed = self.last_speed = 0.0
        self.last_gap_angle = self.last_steer = self.last_log = 0.0
        self.obstacle_history = []

        self.recovery_mode = 'DRIVE'
        self.recovery_started = 0.0
        self.reverse_steer = 0.0
        self.blocked_since = None
        self.collision_active = False
        self.last_collision_at = float('-inf')

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/ego_racecar/collision', self.collision_cb, 10)
        self.create_timer(CONTROL_PERIOD, self.control_loop)

        self.get_logger().info(f'Stage 2 controller started at {MAX_SPEED:.2f} m/s max.')

    def odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if self.pose is not None and math.hypot(p.x - self.pose[0], p.y - self.pose[1]) > 2.0:
            self.obstacle_history.clear()
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.last_odom = time.monotonic()
        self.actual_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def scan_cb(self, msg):
        self.scan = msg
        self.last_scan = time.monotonic()

    def collision_cb(self, msg):
        now, contact = time.monotonic(), bool(msg.data)
        if (contact and not self.collision_active and self.recovery_mode == 'DRIVE'
                and now - self.last_collision_at > COLLISION_DEBOUNCE):
            self.last_collision_at = now
            self.recovery_mode = 'ESCAPE'
            self.recovery_started = now
            self.reverse_steer = -self.last_steer
            self.get_logger().warn('Collision detected. Recovering.')
        self.collision_active = contact

    def command(self, speed, steering=0.0):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(clamp(speed, REVERSE_SPEED, MAX_SPEED))
        msg.drive.steering_angle = float(clamp(steering, -MAX_STEER, MAX_STEER))
        self.drive_pub.publish(msg)
        self.last_speed = msg.drive.speed
        if speed >= 0:
            self.last_steer = msg.drive.steering_angle

    def goal_heading(self):
        x, y, yaw = self.pose

        for _ in range(len(WAYPOINTS)):
            wx, wy = WAYPOINTS[self.waypoint_index]
            if math.hypot(wx - x, wy - y) > WAYPOINT_RADIUS:
                break
            self.waypoint_index = (self.waypoint_index + 1) % len(WAYPOINTS)
            self.get_logger().info(f'Heading to waypoint {self.waypoint_index + 1}')

        i = self.waypoint_index
        wx, wy = WAYPOINTS[i]
        target = math.atan2(wy - y, wx - x)

        self.goal_weight = 1.35
        self.bottom_segment = (i == 0 and y < 1.25 and x < 9.40)
        if self.bottom_segment:
            target = 0.0
        if i == 1 and -11.6 < x < -5.5:
            target, self.goal_weight = math.atan2(9.00 - y, -2.0), 3.0
        elif self.bottom_segment and -4.0 < x <= 3.0:
            target, self.goal_weight = math.atan2(-0.10 - y, 1.0), 4.0
        elif self.bottom_segment and x > 3.0:
            lane_y = -0.81 if x < 8.5 else -0.55
            target, self.goal_weight = math.atan2(lane_y - y, 0.8), 6.0

        return wrap_angle(target - yaw)

    def find_gap(self, desired_angle):
        scan = self.scan
        if scan is None or abs(scan.angle_increment) < 1e-8:
            return None, 0.0, 0.0

        beams = []
        for i, v in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) > math.radians(85):
                continue
            if not math.isfinite(v):
                d = scan.range_max if v > 0 else 0.0
            elif v < scan.range_min:
                d = 0.0
            else:
                d = min(v, scan.range_max)
            beams.append((angle, d))
        if len(beams) < 10:
            return None, 0.0, 0.0

        beams.sort()
        angles = [b[0] for b in beams]
        ranges = [b[1] for b in beams]
        step = abs(scan.angle_increment)

        clearance = BUBBLE_RADIUS
        if self.pose is not None:
            x, y, _ = self.pose
            if -1.0 < x < 3.0 and -1.0 < y < 1.0:
                clearance = 0.165
            elif 3.0 <= x < 9.0 and -1.1 < y < 1.0:
                clearance = 0.17

        velocity = self.actual_speed
        horizon = clamp(0.10 + 0.08 * velocity
                        + velocity ** 2 * (0.03 + 1.0 / (2.0 * BRAKING_ACCEL)), 0.35, 6.4)

        n = len(scan.ranges)
        obstacles = []
        for i in range(0, n, 2):
            d = scan.ranges[i]
            if math.isfinite(d) and d >= scan.range_min and 0 < i < n - 1:
                nb = scan.ranges[i - 1:i + 2]
                if all(math.isfinite(v) and v >= scan.range_min for v in nb):
                    d = sorted(nb)[1]
            if not math.isfinite(d) or d < scan.range_min:
                if d == float('inf'):
                    continue
                d = 0.0
            if d < max(1.8, horizon + 0.7):
                a = scan.angle_min + i * scan.angle_increment
                obstacles.append((LIDAR_OFFSET + d * math.cos(a), d * math.sin(a)))

        # Remember recent walls (rear corners are blind)
        if self.pose is not None:
            x, y, yaw = self.pose
            cs, sn = math.cos(yaw), math.sin(yaw)
            now = time.monotonic()
            history = [(t, c) for t, c in self.obstacle_history if now - t < 1.2]
            rear = {}
            for _, cloud in history:
                for wx, wy in cloud:
                    px = cs * (wx - x) + sn * (wy - y)
                    py = -sn * (wx - x) + cs * (wy - y)
                    if -0.6 < px < 0.15 and abs(py) < 0.65:
                        rear[(round(px / 0.025), round(py / 0.025))] = (px, py)
            if not history or now - history[-1][0] > 0.15:
                history.append((now, [(x + cs * px - sn * py, y + sn * px + cs * py)
                                      for px, py in obstacles]))
            self.obstacle_history = history
            obstacles.extend(rear.values())

        inflated = list(ranges)
        for i, d in enumerate(ranges):
            if d >= max(4.5, velocity * 1.2 + 1.0):
                continue
            pad = math.ceil(math.atan2(clearance, max(d, 0.05)) / step)
            for j in range(max(0, i - pad), min(len(inflated), i + pad + 1)):
                inflated[j] = min(inflated[j], d)
        ranges = inflated
        blocked = [d < GAP_CLEARANCE for d in ranges]

        frontal = [d * math.cos(a) for a, d in beams
                   if abs(a) < math.radians(10) and abs(d * math.sin(a)) < 0.16]
        front_distance = min(frontal) if frontal else 0.0

        candidates = []
        i = 0
        while i < len(blocked):
            if blocked[i]:
                i += 1
                continue
            start = i
            while i < len(blocked) and not blocked[i]:
                i += 1
            end = i
            if (end - start) * step < math.radians(2):
                continue
            margin = min(math.ceil(math.radians(3) / step), (end - start - 1) // 3)
            for j in range(start + margin, end - margin):
                a, d = angles[j], ranges[j]
                if (abs(desired_angle) > math.radians(60)
                        and a * desired_angle < 0.0 and abs(a) > math.radians(10)):
                    continue
                score = (-self.goal_weight * abs(wrap_angle(a - desired_angle))
                         + 2.2 * min(d, 4.0) / 4.0
                         - 0.08 * abs(a)
                         - 0.30 * abs(a - self.last_gap_angle))
                candidates.append((score, a, d))

        max_curv = math.tan(MAX_STEER) / WHEELBASE
        best_angle, best_distance = None, 0.0
        self.creep_only = False
        previews = [horizon] + ([min(horizon, 0.15)] if velocity < 0.55 else [])
        for plan in previews:
            checked = set()
            for _, a, d in sorted(candidates, reverse=True):
                curv = clamp(2.0 * math.sin(a) / self.lookahead, -max_curv, max_curv)
                if abs(curv) * velocity ** 2 > LATERAL_ACCEL * 1.25:
                    continue
                key = round(curv, 3)
                if key in checked:
                    continue
                checked.add(key)
                safe = True
                steps = max(4, math.ceil(plan / 0.20))
                for k in range(steps + 1):
                    travel = plan * k / steps
                    h = curv * travel
                    cs, sn = math.cos(h), math.sin(h)
                    cx = sn / curv if abs(curv) > 1e-4 else travel
                    cy = (1.0 - cs) / curv if abs(curv) > 1e-4 else 0.0
                    side = min(clearance, 0.165 + 0.12 * travel)
                    for px, py in obstacles:
                        lon = cs * (px - cx) + sn * (py - cy)
                        lat = -sn * (px - cx) + cs * (py - cy)
                        if -0.14 < lon < 0.47 + 0.05 * travel and abs(lat) < side:
                            safe = False
                            break
                    if not safe:
                        break
                if safe:
                    best_angle, best_distance = a, d
                    break
            if best_angle is not None:
                self.creep_only = plan < horizon
                break

        return best_angle, best_distance, front_distance

    def control_loop(self):
        now = time.monotonic()

        if (self.pose is None or self.scan is None
                or self.last_odom is None or self.last_scan is None
                or now - self.last_odom > SENSOR_TIMEOUT
                or now - self.last_scan > SENSOR_TIMEOUT):
            self.command(0.0)
            return

        elapsed = now - self.recovery_started

        if self.recovery_mode == 'ESCAPE':
            angle, _, _ = self.find_gap(self.goal_heading())
            if angle is not None:
                steer = math.atan2(2.0 * WHEELBASE * math.sin(angle), self.lookahead)
                self.command(0.35 if self.creep_only else 0.50, steer)
                self.recovery_mode = 'DRIVE'
                self.blocked_since = None
                return
            if self.actual_speed > 0.20:
                self.command(0.0, self.last_steer)
                return
            self.recovery_mode = 'REVERSE'
            self.recovery_started = now
            elapsed = 0.0

        if self.recovery_mode == 'BRAKE':
            self.command(0.0)
            if elapsed >= BRAKE_TIME:
                self.recovery_mode, self.recovery_started = 'REVERSE', now
            return

        if self.recovery_mode == 'REVERSE':
            ranges = self.scan.ranges
            rear_clear = len(ranges) >= 10
            curv = math.tan(self.reverse_steer) / WHEELBASE
            for travel in (-0.10, -0.25, -0.42):
                h = curv * travel
                cs, sn = math.cos(h), math.sin(h)
                cx = sn / curv if abs(curv) > 1e-4 else travel
                cy = (1.0 - cs) / curv if abs(curv) > 1e-4 else 0.0
                for i in range(0, len(ranges), 2):
                    d = ranges[i]
                    if d == float('inf'):
                        continue
                    if not math.isfinite(d) or d < self.scan.range_min:
                        rear_clear = False
                        break
                    if d > 1.5:
                        continue
                    a = self.scan.angle_min + i * self.scan.angle_increment
                    px, py = LIDAR_OFFSET + d * math.cos(a), d * math.sin(a)
                    lon = cs * (px - cx) + sn * (py - cy)
                    lat = -sn * (px - cx) + cs * (py - cy)
                    if -0.14 < lon < 0.47 and abs(lat) < 0.18:
                        rear_clear = False
                        break
                if not rear_clear:
                    break
            self.command(REVERSE_SPEED if rear_clear else 0.0, self.reverse_steer)
            if elapsed >= REVERSE_TIME:
                self.recovery_mode = 'DRIVE'
                self.last_gap_angle = 0.0
            return

        desired = self.goal_heading()
        self.lookahead = clamp(0.80 + 0.15 * max(self.actual_speed - 0.8, 0.0),
                               LOOKAHEAD_MIN, LOOKAHEAD_MAX)

        gap_angle, gap_distance, front_distance = self.find_gap(desired)

        if gap_angle is None:
            previous_steer = self.last_steer
            self.command(0.0)
            if self.blocked_since is None:
                self.blocked_since = now
                self.reverse_steer = -previous_steer
            elif now - self.blocked_since > STUCK_TIMEOUT:
                self.recovery_mode, self.recovery_started = 'BRAKE', now
                self.blocked_since = None
            return

        self.blocked_since = None
        self.last_gap_angle = gap_angle

        steering = clamp(math.atan2(2.0 * WHEELBASE * math.sin(gap_angle), self.lookahead),
                         -MAX_STEER, MAX_STEER)

        curv = abs(math.tan(steering) / WHEELBASE)
        corner_speed = math.sqrt(LATERAL_ACCEL / max(curv, 0.01))
        dist = max(0.0, min(gap_distance, front_distance) - 0.25 - 0.22 * self.actual_speed)
        braking_speed = math.sqrt(2.0 * BRAKING_ACCEL * dist)
        speed = min(MAX_SPEED, corner_speed, braking_speed)
        if self.bottom_segment:
            speed = min(speed, BOTTOM_SPEED)
            if -4.0 < self.pose[0] < 9.0:
                speed = min(speed, CHICANE_SPEED)
        if gap_distance < 1.5:
            speed = min(speed, CLOSE_GAP_SPEED)
        if self.creep_only:
            speed = min(speed, CREEP_SPEED)
        speed = min(speed, max(0.30, self.last_speed) + ACCEL_LIMIT * CONTROL_PERIOD)

        self.command(speed, steering)

        if now - self.last_log > 3.0:
            self.get_logger().info(
                f'Waypoint {self.waypoint_index + 1}/{len(WAYPOINTS)} | '
                f'Gap {math.degrees(gap_angle):+.0f} deg | Speed {speed:.2f} m/s')
            self.last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = Stage2Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.command(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()