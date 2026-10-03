# Omnidirectional Mecanum Robot — ROS 2 Jazzy

Autonomous omnidirectional (mecanum-wheeled) robot with SLAM and Nav2 navigation.

| Machine | Role |
|---|---|
| Arduino Mega 2560 | Motor control (PD loop) and encoder reading |
| Raspberry Pi 5 (`omni-pi`) | Hardware bringup: serial bridge to Arduino, IMU, lidar, joystick teleop |
| Laptop | SLAM Toolbox, Nav2, RViz |

Packages: `omniwheel_complete` (description, config, launch, maps) and `serial_interface` (Arduino serial node, IMU node).

---

## 1. Setup

### 1.1 Laptop with Docker (recommended)

The laptop side runs in Docker, so any Linux laptop works regardless of what ROS version (if any) is installed on it. The environment lives in a separate repo, [`omnidirectional_robot_docker`](https://github.com/nishanmud2023/omnidirectional_robot_docker), which clones this repo automatically.

> Requires Linux (Docker on Windows/macOS does not support the host networking ROS needs).

**One-time setup**

```bash
# 1. Install Docker — skip this step if `docker --version` already prints a version
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
# → log out and log back in, then test:
docker run hello-world

# 2. Clone the Docker repo
cd ~
git clone git@github.com:nishanmud2023/omnidirectional_robot_docker.git
cd omnidirectional_robot_docker

# 3. Create the config file (not stored in git)
echo "APP_NAME=omni-robot" > .env

# 4. Build the image (~5 min on a good connection; avoid slow WiFi)
source docker/build_docker.sh
```

**Every time**

```bash
cd ~/omnidirectional_robot_docker
source docker/run_docker.sh          # starts container, pulls latest robot_ws
```

Inside the container (prompt `root@...:/workspace#`):

```bash
cd /workspace/src/robot_ws
colcon build --symlink-install       # only after code changes
source install/setup.bash
```

Extra terminal into the running container:

```bash
cd ~/omnidirectional_robot_docker
source docker/into_docker.sh
source /workspace/src/robot_ws/install/setup.bash
```

Type `exit` in the first terminal to stop the container. Files in `workspace/` persist on the laptop.

**Notes**
- Inside the container, `~` is `/root` and is **not** kept after exit. Save maps, bags and anything else under `/workspace/...`.
- Do git commit/push on the laptop terminal, not inside the container.
- Adding a package: add it to `docker/scripts/install_system.sh` in the Docker repo and rebuild.

### 1.2 Raspberry Pi (native, no Docker)

The Pi runs ROS 2 Jazzy natively on Ubuntu 24.04.

**Connect to the Pi over SSH.** The laptop and Pi must be on the same network. By default the Pi connects to the lab WiFi (`robot-ap`), so connect the laptop to the same network, then:

```bash
ssh omnirobot@omni-pi.local
```

User: `omnirobot`. Ask the maintainer for the password.

If `omni-pi.local` is not found, use the Pi's IP address instead (`ssh omnirobot@<IP>`). Find it with `hostname -I` on the Pi, or with the network scan in [section 3](#3-pi-networking).

**Update and build:**

```bash
cd ~/robot_ws
git pull
colcon build --symlink-install
source install/setup.bash
```

### 1.3 Network check (both machines)

```bash
echo $ROS_DOMAIN_ID     # must be 42 on both (already set in Docker image and Pi ~/.bashrc)
hostname -I             # shows this machine's IP address
```

---

## 2. Running

Paths below are for the **Docker** container. Without Docker, replace `/workspace/src/robot_ws` with `~/robot_ws`.

### 2.1 Simulation

```bash
# SLAM mapping (sim is the default, no argument needed)
ros2 launch omniwheel_complete navigation_with_slam.launch.py

# Navigation on a saved map
ros2 launch omniwheel_complete navigation_localization.launch.py \
  map:=/workspace/src/robot_ws/src/omniwheel_complete/maps/my_map.yaml
```

### 2.2 Real robot

**On the Pi** (over SSH, see [1.2](#12-raspberry-pi-native-no-docker)):

```bash
ros2 launch omniwheel_complete bringupomni_real.launch.py
```

**On the laptop — create a map:**

```bash
ros2 launch omniwheel_complete navigation_with_slam.launch.py use_sim_time:=False
```

**Save the map** (in a second terminal):

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /workspace/src/robot_ws/src/omniwheel_complete/maps/my_map
```

**On the laptop — navigate on a saved map:**

```bash
ros2 launch omniwheel_complete navigation_localization.launch.py \
  use_sim_time:=False \
  map:=/workspace/src/robot_ws/src/omniwheel_complete/maps/hall_map_20260915_143528.yaml
```

Available maps in `src/omniwheel_complete/maps/`:

| Map | File |
|---|---|
| Hall | `hall_map_20260915_143528.yaml` |
| Lab | `lab_map.yaml` |
| Student launch area | `studentlaunch_map_20260917_140033_cleaned.yaml` |

---

## 3. Pi networking

The Pi knows three WiFi connections:

| Connection | Network | Internet |
|---|---|---|
| `robot-ap` | Lab WiFi (default, connects automatically) | Yes |
| `robot-LEC` | Modem | No |
| `robot-hotspot` | Your laptop's hotspot | Depends on laptop |

Check the Pi's date first after boot. The Pi has no battery-backed clock, so the date is only correct once it has synced over a network with internet:

```bash
date
```

The WiFi fallback setup script is stored at:

```bash
cat ~/migrate-to-networkmanager.sh
```

**Switch connections** (run on the Pi):

```bash
sudo nmcli connection up robot-ap      # lab WiFi with internet
sudo nmcli connection up robot-LEC     # modem without internet
```

Switching networks drops your SSH session. Reconnect from the laptop once both are on the new network.

### 3.1 Switch to the laptop hotspot safely over SSH

Turn on the hotspot on the laptop first. Then run this on the Pi. It schedules the switch 20 s later (so your SSH command finishes cleanly), retries for about 90 s, and logs the result to `~/hotspot.log`:

```bash
sudo systemd-run --on-active=20sec --unit=go-hotspot bash -c 'for i in $(seq 1 18); do nmcli dev wifi rescan 2>/dev/null; sleep 3; nmcli -f SSID,CHAN,SIGNAL dev wifi list >> /home/omnirobot/hotspot.log; nmcli connection up robot-hotspot >> /home/omnirobot/hotspot.log 2>&1 && exit 0; sleep 2; done'
```

### 3.2 Check that the Pi joined the hotspot

On the laptop (install once with `sudo apt install nmap`):

```bash
hostname -I                  # find the hotspot address, usually 10.42.0.1
sudo nmap -sn 10.42.0.0/24   # replace 10.42.0 with the first three numbers of that address
```

It should report **at least 2 hosts up**: the laptop and the Pi. The IP that isn't the laptop's is the Pi. SSH to it with `ssh omnirobot@<that IP>` if `omni-pi.local` doesn't resolve.

---

## 4. Motion tests with rosbag

Record continuously during normal runs, or follow these steps for specific odometry tests.

### 4.1 Start recording

Use a different `TEST` name for each test: `straight_1m`, `lateral_1m`, `rot_90`, `diag_1m`, `straight_10s`.

**Pi, terminal 1:**

```bash
mkdir -p ~/bags
TEST=straight_1m
ros2 bag record -o ~/bags/${TEST}_pi_$(date +%H%M%S) \
  /wheel/commands /wheel/states /wheel/pwm /wheel/pd_error \
  /wheel/odometry /odometry/filtered /imu/data_raw /scan /cmd_vel /tf /tf_static
```

**Laptop (inside Docker):**

```bash
mkdir -p /workspace/bags
TEST=straight_1m
ros2 bag record -o /workspace/bags/${TEST}_laptop_$(date +%H%M%S) \
  /map /plan /local_plan /odometry/filtered /tf /tf_static
```

On the laptop this ends up in `~/omnidirectional_robot_docker/workspace/bags`.

### 4.2 Drive the test

Each command publishes at 10 Hz for a fixed number of messages, then sends a stop.

| Test | Motion | Command |
|---|---|---|
| `straight_1m` | 1 m forward (0.2 m/s × 5 s) | `ros2 topic pub -r 10 --times 50 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"` |
| `lateral_1m` | 1 m left (0.2 m/s × 5 s; `y: -0.2` for right) | `ros2 topic pub -r 10 --times 50 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.2}}" && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"` |
| `rot_90` | 90° CCW (0.31416 rad/s × 5 s; negative for CW) | `ros2 topic pub -r 10 --times 50 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.31416}}" && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"` |
| `diag_1m` | 1 m front-left (0.1414 m/s in x and y × 5 s) | `ros2 topic pub -r 10 --times 50 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1414, y: 0.1414}}" && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"` |
| `straight_10s` | ≈ 2 m forward (0.2 m/s × 10 s) — make sure there is room | `ros2 topic pub -r 10 --times 100 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"` |

### 4.3 After each test

1. Wait until the robot has fully stopped.
2. Measure the real distance or angle (tape measure / protractor) and note it with the test name.
3. Press `Ctrl+C` in both recording terminals.
4. Check the Pi recording:
   ```bash
   ros2 bag info ~/bags/<folder_name>
   ```

### 4.4 Copy Pi recordings to the laptop

Run on the laptop (outside Docker):

```bash
scp -r omnirobot@omni-pi.local:~/bags ~/bags_from_pi
```

Diagnosis scripts and earlier test data are in `src/omniwheel_complete/Diagnosis/`.

---

## 5. Troubleshooting: clean restart

Use this when nodes freeze, topics act strangely, or a previous session didn't shut down properly (e.g. after the battery died). Always try `Ctrl+C` in the launch terminal first; use `pkill -9` only if a terminal is frozen.

### 5.1 Pi

**See what's still running:**

```bash
ps aux | grep -E "ros2|serial_node|mpu6050_node|ekf_node|rplidar|robot_state" | grep -v grep
```

**Kill everything:**

```bash
pkill -9 -f "ros2 launch"
pkill -9 -f robot_state_publisher
pkill -9 -f serial_node
pkill -9 -f mpu6050_node
pkill -9 -f rplidar_composition
pkill -9 -f ekf_node
```

If `ps aux` still shows something, kill it directly by its PID: `kill -9 <PID>`. A leftover `ros2-daemon` process is normal and fine.

**Check the Arduino serial port is free** (should print nothing):

```bash
lsof /dev/ttyACM0
```

### 5.2 Laptop

```bash
pkill -9 -f "ros2 launch"
pkill -9 -f rviz2
```

With Docker, `exit` in the first container terminal also stops everything inside it.

### 5.3 Both machines

**Restart the ROS discovery daemon** to clear stale cached data:

```bash
ros2 daemon stop
ros2 daemon start
```

**Relaunch in order:** Pi bringup first, then the laptop launch. Then check, in a new laptop terminal:

```bash
echo $ROS_DOMAIN_ID     # must be 42
ros2 topic list         # should show topics from both machines within a few seconds
ros2 node list          # check that a specific node is alive, e.g. | grep bt_navigator
```

If `ros2 topic list` stays empty, check that both machines are on the same network and `ROS_DOMAIN_ID` matches.

---

## 6. Working with Git

GitHub is the single source of truth. The laptop and the Pi each have their own copy of this repo and stay in sync **only** through push and pull.

### 6.1 Where to edit and push from

| Copy | Location | Edit & push here? |
|---|---|---|
| Laptop (native) | `~/robot_ws` | ✅ Yes — your main working copy on the laptop |
| Pi | `~/robot_ws` on `omni-pi` | ✅ Yes — for Pi-side changes (bringup, serial, EKF) |
| Laptop Docker copy | `~/omnidirectional_robot_docker/workspace/src/robot_ws` (`/workspace/src/robot_ws` inside the container) | ❌ No — treat as read-only; it updates itself from GitHub |

Why not push from the Docker copy:
- Its files are created by the container as `root`, so git on the laptop hits permission errors there.
- The container has no GitHub login or git identity.
- The container runs `git pull` at every start. Local commits in this copy can make that pull fail and **stop the container from starting**.

So the flow on the laptop is: edit in `~/robot_ws` → push → the Docker copy picks it up. To get changes into an already running container without restarting it:

```bash
# inside the container
cd /workspace/src/robot_ws && git pull
colcon build --symlink-install
```

Always run `git commit` / `git push` in a normal terminal on the laptop or Pi (prompt `nishan@...$` or `omnirobot@...$`), never inside the container (prompt `root@...#`).

### 6.2 Daily routine

**Before you start working** (on whichever machine):

```bash
cd ~/robot_ws
git pull
```

**After making changes:**

```bash
cd ~/robot_ws
git status                         # see what changed
git diff path/to/file              # optional: review a change
git add path/to/file1 path/to/file2
git commit -m "Short description of what and why"
git push
```

Add files by name rather than `git add -A`, so local maps, test outputs and other clutter don't end up on GitHub. `build/`, `install/`, `log/` and `__pycache__/` are already ignored.

**Before switching machines** (e.g. you fixed something on the Pi and now continue on the laptop): push on the first machine, then pull on the second. Unpushed changes on one machine are invisible to the other.

### 6.3 If push is rejected

`! [rejected] ... (fetch first)` means the other machine pushed something you don't have yet:

```bash
git pull --rebase
git push
```

If `git pull --rebase` reports a **conflict** (both machines changed the same lines), git marks the conflicting parts in the file. Edit the file to keep the right version, then:

```bash
git add path/to/file
git rebase --continue
git push
```

To back out of a rebase that went wrong: `git rebase --abort`.

### 6.4 Setting up git on a new machine

GitHub doesn't accept account passwords for git. Each machine needs its own SSH key:

```bash
ssh-keygen -t ed25519                # press Enter at every prompt
cat ~/.ssh/id_ed25519.pub            # copy this line
```

Add it on GitHub under **Settings → SSH and GPG keys → New SSH key**, then test:

```bash
ssh -T git@github.com                # "Hi <user>! You've successfully authenticated"
```

Set your identity once per machine:

```bash
git config --global user.name "Your Name"
git config --global user.email "your-github-email"
```

Clone with the SSH address (`git@github.com:...`). For an existing clone that uses `https://`, switch it:

```bash
git remote set-url origin git@github.com:nishanmud2023/robot_ws.git
```
