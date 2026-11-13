# SUAVE Drone Control Workspace (ROS 2 Humble + PX4)

Offboard flight stack: `suave_main` connects to PX4 over MAVSDK, feeds RealSense + RTAB-Map visual-inertial odometry into PX4 as external vision, and flies from console commands.

This branch (`logging-disturbances`) adds:
- **Flight data logging, always on**: every MAVSDK telemetry stream, every setpoint and command, a rosbag of all ROS 2 topics and the PX4 ULog, in one folder per flight under `data/`.
- **Wind and external disturbances in simulation**: a Gazebo plugin applies steady wind, turbulence, gusts, forces and torques to the drone, and logs them together with Gazebo ground truth.

```
src/suave_launch/
├── launch/suave_slam.py           RTAB-Map RGB-D odometry + IMU filter (started by suave_main)
└── src/
    ├── suave_main.cpp             entry point, Ctrl-C = land and exit
    ├── controllers/               SuaveFlightController (console commands), ControllerMacros
    ├── logging/                   FlightLogger (all telemetry -> CSV, rosbag, ULog), CsvWriter
    ├── mavutil/                   PX4 connection (MavUtil), Drone wrapper, TelemetryProperty
    ├── vio/VIOBridge              RTAB-Map /odom -> PX4 Mocap (vision position + odometry)
    ├── ros/RosNodeSpinner         multithreaded executor task
    ├── common/                    task / process helpers
    └── util/                      console logging macros, Result, Timer
src/suave_gz_disturbance/          Gazebo Harmonic plugin: wind, turbulence, gusts, forces, torques
tools/
├── sim.sh                         start Gazebo (with the plugin) + PX4 SITL
├── disturb.py                     send disturbances / play scenarios
├── scenarios/*.txt                ready-made disturbance sequences
└── plot_flight.py                 plot a logged flight
data/                              flight logs (not committed)
```

## Setup (companion computer, Ubuntu 22.04)
```
sudo apt install git -y
mkdir ~/Dev && cd ~/Dev
git clone https://github.com/Destiny-Dragon-604/dronecpp-ros2_ws.git
cd dronecpp-ros2_ws/setup
source setup.bash      # ROS 2 Humble, RealSense, RTAB-Map, MAVSDK 1.4.16, FTDI id; reboot after
```

## Build
```
ros                    # alias: source /opt/ros/humble/setup.bash
cd ~/Dev/dronecpp-ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source ./install/setup.bash
```

## Fly
The connection is chosen by `SUAVE_MAVLINK_SERIAL_ID`:
- unset or `SITL` -> `udp://:14540`, camera/VIO stack skipped (PX4 SITL has its own estimate)
- FTDI id (set by `setup/find_ftdi_id.pl`) -> `serial:///dev/serial/by-id/<id>:57600`, camera/VIO stack started

**Simulation:** in one terminal
```
~/Dev/dronecpp-ros2_ws/tools/sim.sh            # or: tools/sim.sh baylands, tools/sim.sh --headless
```
then in another
```
ros2 run suave_launch suave_main
```
(`make px4_sitl gz_x500` in PX4-Autopilot still works, but without the disturbance plugin.)

**Real drone:** put the flight controller in **manual** mode (PX4 will refuse offboard otherwise), run `ros2 run suave_launch suave_main`, walk the drone around to build the initial RTAB-Map map, press enter, then command it.

### Commands (NED, metres, down < 0 is up)
| command | action |
|---|---|
| `takeoff [alt]` | arm, enter offboard, climb to `alt` above the takeoff point (default 1.75, max 5) |
| `move dn de dd` | step relative to the current position (max 5 m) |
| `goto n e d` | fly to a point relative to the takeoff point (max 5 m) |
| `hold` | hold the current position |
| `land` | descend at 0.5 m/s, disarm on touchdown (falls back to PX4 land after 30 s) |
| `status` | position, heading, armed / in-air / offboard |
| `exit` | land if airborne, stop camera/VIO, quit |
| `mark <note>` | write a note into the flight's events at this instant |
| `log` | show this flight's data folder |
| `wind`, `turb`, `gust`, `push`, `torque`, `drag`, `calm`, `dstatus` | disturbances, simulation only (see below) |

Ctrl-C at any time lands and exits.

## Flight data
Every run of `suave_main` creates `data/flight_<YYYYmmdd_HHMMSS>/` (override the root with `SUAVE_DATA_DIR`):

| file | contents |
|---|---|
| `position_velocity_ned.csv`, `velocity_ned.csv` | PX4 local position / velocity estimate (NED) |
| `attitude_quaternion.csv`, `attitude_euler.csv`, `angular_velocity_body.csv`, `heading.csv` | attitude and body rates |
| `imu.csv`, `scaled_imu.csv`, `raw_imu.csv` | accelerometer, gyro, magnetometer (body FRD) |
| `odometry.csv`, `vision_odometry.csv` | PX4 odometry; VIO sent to PX4 (real drone) |
| `actuator_outputs.csv`, `actuator_controls.csv` | motor outputs and mixer inputs |
| `global_position.csv`, `home.csv`, `raw_gps.csv`, `gps_info.csv`, `ground_truth.csv` | global position / GPS |
| `battery.csv`, `health.csv`, `flight_mode.csv`, `landed_state.csv`, `armed.csv`, `in_air.csv`, `rc_status.csv`, `status_text.csv`, `scaled_pressure.csv`, `distance_sensor.csv` | system state (state-like streams only on change) |
| `setpoints.csv` | every offboard setpoint sent to PX4 |
| `events.csv` | console commands, results, marks, errors |
| `mavsdk_log.csv` | every MAVSDK log line (the console only shows real warnings/errors and PX4 messages) |
| `disturbance.csv`, `disturbance_events.csv` | SITL: wind, forces, torques and Gazebo ground truth at 100 Hz |
| `px4_*.ulg` | SITL: the full PX4 log (every uORB topic) |
| `rosbag/` | all ROS 2 topics (`SUAVE_ROSBAG=0` to disable) |
| `metadata.txt`, `summary.txt` | branch/commit, PX4 version, rows per file |

All CSVs share `t_unix` (wall clock, s) and `t` (s since start), so they line up with each other and with `disturbance.csv`.

Rates: ~30 Hz for state and actuators, 50 Hz IMU in SITL (each message costs a callback on MAVSDK's single user thread; the PX4 ULog has everything at full rate). On the drone the PX4 link is a 57600 baud serial line shared with VIO and offboard traffic, so the logger asks for ~10 Hz there. For high-rate data from real flights use the ULog on the flight controller's SD card (download with QGroundControl).

Plot a flight (newest by default):
```
python3 tools/plot_flight.py [data/flight_...] [--show]
```
It writes position, track, tracking error, velocity, attitude, rates, IMU, motor and disturbance plots to `<flight>/plots/` and prints RMS / max tracking error. For the ULog: `ulog2csv data/flight_.../px4_*.ulg` (pyulog) or https://review.px4.io.

## Wind and disturbances (simulation)
Start the sim with `tools/sim.sh`, then type disturbances directly in `suave_main` or use `tools/disturb.py` from any terminal. All vectors are NED; wind is the velocity of the air (the direction it blows toward).

| command | effect |
|---|---|
| `wind n e d` | steady wind, m/s (`wind 0 5 0`: 5 m/s toward east) |
| `turb sigma [L] [sigma_d]` | turbulence: first-order Gauss-Markov per axis (Dryden-like), sigma m/s, length scale L m (20), vertical sigma (sigma/2) |
| `gust n e d dur` | 1-cosine gust peaking at (n, e, d) m/s over `dur` s |
| `push n e d dur` | external force, N, for `dur` s |
| `torque r p y dur` | external torque about body roll/pitch/yaw (FRD), N m, for `dur` s |
| `drag cd a_xy a_z` | body drag model (default 1.0, 0.05 m², 0.12 m²) |
| `calm` | remove everything |
| `dstatus` (suave_main) / `status` (disturb.py) | show the current state |

Scenarios for repeatable tests (lines of `<seconds> <command>`):
```
python3 tools/disturb.py run tools/scenarios/gust_rejection.txt
```
Included: `steady_wind`, `gust_rejection`, `turbulence`, `impulse_push`.

Model: wind acts through body drag, $D(u) = -\tfrac12 \rho\, |u|\, (C_d A) \odot u$, applied as $D(v - w) - D(v)$ using the drone's true velocity $v$ and the wind $w$. Only the extra force due to wind is added, so with no wind the simulation is identical to stock PX4. Turbulence time constant is $T = L / \max(|\bar w|, 1\ \text{m/s})$, so use it together with a mean wind for realistic spectra. The plugin attaches to `x500_0::base_link`; parameters are in `src/suave_gz_disturbance/config/server.config`.

To watch the map over ssh (`ssh -YC suave@<ip>`), run the SLAM launch by hand with `viz:=true`.

## Troubleshooting: `offboard start failed: Command Denied`
1. The drone must be in manual mode when the script starts.
2. The RealSense must be on a USB 3 port/cable (blue port); check with `realsense-viewer`.
3. If the RealSense works, run RTAB-Map on its own and confirm it opens:
   ```
   ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true
   ros2 param set /camera/camera depth_module.emitter_enabled 0
   ros2 launch suave_launch suave_slam.py viz:=true
   ```
4. Bad FTDI link: check `ls /dev/serial/by-id/` matches `$SUAVE_MAVLINK_SERIAL_ID`, reseat the board and reboot. After swapping boards:
   ```
   echo 'export SUAVE_MAVLINK_SERIAL_ID=usb-FTDI_FT232R_USB_UART_<NEW_ID>-if00-port0' >> ~/.bashrc
   source ~/.bashrc
   ```
