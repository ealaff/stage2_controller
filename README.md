# Stage 2 — F1TENTH Autonomous Controller

Waypoint-guided Follow-the-Gap controller for the official KFUPM simulator's
`levine_obs` map, using ROS 2 Jazzy.

## Verified results

Final three-waypoint configuration, tested on 19 November 2026:

| Metric | Official / measured result |
| --- | --- |
| Consecutive completed laps | **3** |
| Session collisions | **0** |
| Lap times | **35.24 s, 29.88 s, 34.58 s** |
| Best lap | **29.88 s** |
| Configured maximum speed | **6.00 m/s** |
| Measured peak speed | **5.40 m/s** |

The first finish-line crossing completes the out-lap; the results above are
three officially counted completed laps. Speed is adaptive: the 6 m/s setting
is a ceiling, not a constant speed through corners and obstacles.

The recorded test started at `(10.15, 0.221)`, facing north
(`yaw = pi / 2`), using the simulator's supported `/initialpose` reset.
Use the same start pose when comparing lap times. A pose reset clears the
official session counters.

## Navigation and waypoints

The controller retains waypoint-guided Follow-the-Gap navigation, LiDAR
obstacle avoidance, and Ackermann steering. The explicit bottom-right waypoint
has been removed. The remaining waypoints are visited in this order:

```python
WAYPOINTS = [
    (9.80, 8.90),        # Top-right
    (-13.60, 8.95),      # Top-left
    (-13.65, 0.25),      # Bottom-left
]
```

After the bottom-left waypoint, bottom-corridor guidance closes the loop and
returns the car toward the first waypoint. Approach guidance aligns the car
before the central chicane, oval obstacle, and upper-left projecting wall.

Gap selection combines waypoint direction, available depth, and steering
continuity. A swept-body check rejects turns that would clip an obstacle even
when the selected LiDAR ray is clear. Three-beam median filtering and short
LiDAR history reduce noisy clearance decisions and protect the rear corners.

## Current settings

All controller settings are in
[`stage2_controller/controller.py`](stage2_controller/controller.py).

| Setting | Value |
| --- | --- |
| Maximum speed | 6.00 m/s |
| Lower-straight speed cap | 2.40 m/s |
| Dense lower-obstacle section cap | 1.85 m/s |
| Selected gap shorter than 1.5 m | 1.35 m/s cap |
| Short-path crawl fallback | 0.45 m/s cap |
| Commanded acceleration ramp | 3.5 m/s² |
| Braking assumption | 4.0 m/s² |
| Corner lateral-acceleration limit | 3.2 m/s² |
| Control rate | 40 Hz |
| Wheelbase / maximum steering | 0.3302 m / 0.40 rad |
| Steering lookahead | 0.80–1.45 m, speed-dependent |
| Waypoint switching radius | 0.80 m |
| General obstacle radius | 0.21 m |
| Central chicane / oval-passage clearance | 0.165 m / 0.17 m |
| Gap clearance threshold | 0.48 m |
| Sensor timeout | 0.35 s |

The collision-check preview grows with speed and braking distance, up to
6.4 m. At low speed, a shorter validated path can permit crawling instead of
repeated stopping and reversing. Occasional pauses remain in the tight chicane.

## Collision recovery and stopping

On contact, the controller first checks for a safe forward gap. If one exists,
it continues at 0.50 m/s, or 0.35 m/s when only the shorter crawl path is clear.
If forward travel is blocked, it brakes before reversing; reverse travel is
also checked against the visible obstacle footprint.

Recovery uses a reverse command of -0.45 m/s for up to 0.90 s before retrying
normal navigation. There is no permanent collision latch. Missing or stale
LiDAR/odometry, or no validated movement path, still commands zero speed.

Collision-escape branches were checked with synthetic collision callbacks;
the final three-lap simulation recorded no actual collisions.

## Requirements

- Ubuntu 24.04 and **ROS 2 Jazzy**.
- Existing official KFUPM F1TENTH simulator workspace.
- `rclpy`, `sensor_msgs`, `nav_msgs`, `ackermann_msgs`, and `std_msgs`.

## Build and run

Start the official simulator in one terminal:

```bash
cd ~/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition
./scripts/launch_sim.sh
```

In a separate Jazzy terminal, stop any existing controller or teleoperation
publisher before starting this controller. Only one controller should publish
to `/drive`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition/starter_ws/install/setup.bash

cd ~/controller_ws
/usr/bin/python3 /usr/bin/colcon build --symlink-install --packages-select stage2_controller
source install/setup.bash

# Expected map: maps/levine_obs
ros2 param get /bridge map_path

ros2 run stage2_controller controller
```

To reproduce the recorded start pose, publish this **before** running the
controller, with the simulator active and the car stopped:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 10.15, y: 0.221, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.70710678, w: 0.70710678}}}}"
```

After exiting the controller, the simulator's existing stop command is:

```bash
~/CIE-KFUPM-F1tenth-Autonomous-Racing-Competition/scripts/stop_car.sh
```

## ROS interface

| Topic | Role |
| --- | --- |
| `/scan` | LiDAR input |
| `/ego_racecar/odom` | Pose and measured velocity |
| `/ego_racecar/collision` | Collision input |
| `/drive` | Ackermann command output |
| `/competition/lap_count` | Official completed laps |
| `/competition/collision_count` | Official session collisions |
| `/competition/last_lap_time` | Last completed lap time |
| `/competition/best_lap_time` | Best completed lap time |
| `/competition/lap_history` | Official completed lap times |

For example:

```bash
ros2 topic echo /competition/lap_count
ros2 topic echo /competition/collision_count
ros2 topic echo /competition/best_lap_time
```

## Validation and scope

Python syntax, the ROS 2 Jazzy package build, three-point loop closure,
collision-escape logic, reverse checks, and sensor fail-safes passed validation.

The optimization changed only `stage2_controller/controller.py`; this README
was then updated to document the final configuration. Simulator source, maps,
launch files, package configuration, and competition scoring were not edited.
