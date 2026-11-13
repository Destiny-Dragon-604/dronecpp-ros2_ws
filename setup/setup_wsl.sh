#!/usr/bin/env bash
# Drone Controls dev environment for WSL2 Ubuntu 22.04
#   ROS 2 Humble, MAVSDK 1.4.16, PX4 SITL + Gazebo Harmonic, GitHub over SSH,
#   and the dronecpp-ros2_ws workspace built in ~/Dev.
#
# Run inside Ubuntu (not as root), from a copy of this repo (e.g. on the Windows side):
#   bash /mnt/c/.../dronecpp-ros2_ws/setup/setup_wsl.sh
# Safe to re-run: finished steps are skipped.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WIN_REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG="$SCRIPT_DIR/setup.log"
DEV="$HOME/Dev"
WS="$DEV/dronecpp-ros2_ws"
PX4_DIR="$DEV/PX4-Autopilot"
PX4_TAG="${PX4_TAG:-v1.16.2}"
GH_SSH="git@github.com:Destiny-Dragon-604/dronecpp-ros2_ws.git"
BRANCH="${BRANCH:-main}"

exec > >(tee -a "$LOG") 2>&1
step() { echo; echo "=================== $* ==================="; echo "[STEP] $*" > "$SCRIPT_DIR/status.txt"; }
done_flag() { [ -f "$HOME/.drone_setup/$1" ]; }
mark() { mkdir -p "$HOME/.drone_setup"; touch "$HOME/.drone_setup/$1"; }

if [ "$(id -u)" = "0" ]; then echo "Run as your normal user, not root (sudo is used where needed)."; exit 1; fi
if [ "$(lsb_release -cs 2>/dev/null)" != "jammy" ]; then echo "This script expects Ubuntu 22.04 (jammy)."; exit 1; fi

echo "Started $(date)"
sudo -v

# ---------------------------------------------------------------- 1. base tools
step "1/8 Base packages"
if ! done_flag base; then
    sudo apt-get update
    sudo apt-get install -y git curl wget build-essential cmake python3-pip openssh-client \
        software-properties-common locales lsb-release gnupg net-tools
    mark base
fi

# ---------------------------------------------------------------- 2. SSH key + GitHub
step "2/8 GitHub SSH key"
if [ ! -f "$HOME/.ssh/id_ed25519" ]; then
    mkdir -p "$HOME/.ssh" && chmod 700 "$HOME/.ssh"
    ssh-keygen -t ed25519 -N "" -C "chirag-wsl-drone" -f "$HOME/.ssh/id_ed25519"
fi
cp "$HOME/.ssh/id_ed25519.pub" "$SCRIPT_DIR/id_ed25519.pub"
ssh-keyscan -t ed25519 github.com >> "$HOME/.ssh/known_hosts" 2>/dev/null
sort -u -o "$HOME/.ssh/known_hosts" "$HOME/.ssh/known_hosts"

gh_ok() { local out; out="$(ssh -o BatchMode=yes -T git@github.com 2>&1 || true)"; [[ "$out" == *"successfully authenticated"* ]]; }
until gh_ok; do
    echo "[WAITING-FOR-KEY]" > "$SCRIPT_DIR/status.txt"
    echo
    echo "Add this public key to GitHub: https://github.com/settings/ssh/new"
    echo "  Title: WSL Drone Controls   Key type: Authentication Key"
    echo
    cat "$HOME/.ssh/id_ed25519.pub"
    echo
    read -r -p "Press Enter once it's added... "
done
echo "GitHub SSH: OK"

if [ -z "$(git config --global user.name)" ]; then git config --global user.name "Chirag"; fi
if [ -z "$(git config --global user.email)" ]; then
    read -r -p "Email for git commits (your GitHub email or noreply address): " GIT_EMAIL
    git config --global user.email "$GIT_EMAIL"
fi
git config --global init.defaultBranch master

# ---------------------------------------------------------------- 3. (repo already on GitHub)
step "3/8 GitHub access checked"

# ---------------------------------------------------------------- 4. ROS 2 Humble
step "4/8 ROS 2 Humble (large download)"
if ! done_flag ros; then
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo add-apt-repository -y universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get upgrade -y
    sudo apt-get install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions
    [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
    rosdep update
    mark ros
fi

# ---------------------------------------------------------------- 5. MAVSDK
step "5/8 MAVSDK 1.4.16"
if ! done_flag mavsdk; then
    sudo apt-get install -y "$WIN_REPO/setup/libmavsdk-dev_1.4.16_ubuntu20.04_amd64.deb" || sudo apt-get -f install -y
    sudo ldconfig
    mark mavsdk
fi

# ---------------------------------------------------------------- 6. PX4 SITL + Gazebo
step "6/8 PX4 $PX4_TAG + Gazebo Harmonic (large download, long build)"
mkdir -p "$DEV"
if [ ! -d "$PX4_DIR/.git" ]; then
    git clone --branch "$PX4_TAG" --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
fi
if ! done_flag px4deps; then
    bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx
    mark px4deps
fi
if ! done_flag px4build; then
    # Build only; "make px4_sitl gz_x500" would also launch the simulator and block here.
    if [ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
        (cd "$PX4_DIR" && make px4_sitl)
    fi
    mark px4build
fi

# ---------------------------------------------------------------- 7. workspace
step "7/8 Clone and build dronecpp-ros2_ws"
if [ ! -d "$WS/.git" ]; then
    git clone --branch "$BRANCH" "$GH_SSH" "$WS"
fi
set +u
source /opt/ros/humble/setup.bash
(cd "$WS" && rosdep install --from-paths src --ignore-src -r -y && colcon build)

# ---------------------------------------------------------------- 8. shell setup
step "8/8 Shell setup and analysis tools"
python3 -m pip install --user --upgrade pandas matplotlib pyulog
grep -q "alias ros=" "$HOME/.bash_aliases" 2>/dev/null || echo "alias ros='source /opt/ros/humble/setup.bash'" >> "$HOME/.bash_aliases"
if ! grep -q "dronecpp-ros2_ws/install" "$HOME/.bashrc"; then
    cat >> "$HOME/.bashrc" <<'EOF'

# Drone Controls workspace
source /opt/ros/humble/setup.bash
[ -f ~/Dev/dronecpp-ros2_ws/install/setup.bash ] && source ~/Dev/dronecpp-ros2_ws/install/setup.bash
export SUAVE_MAVLINK_SERIAL_ID=SITL
export SUAVE_DATA_DIR=~/Dev/dronecpp-ros2_ws/data
export GZ_IP=127.0.0.1
alias sim='~/Dev/dronecpp-ros2_ws/tools/sim.sh'
alias disturb='python3 ~/Dev/dronecpp-ros2_ws/tools/disturb.py'
alias plotflight='python3 ~/Dev/dronecpp-ros2_ws/tools/plot_flight.py'
EOF
fi

echo "[DONE]" > "$SCRIPT_DIR/status.txt"
cat <<EOF

All set ($(date)).
Open two Ubuntu terminals:
  1)  sim                                   (Gazebo + PX4 + disturbance plugin)
  2)  ros2 run suave_launch suave_main      then: takeoff / wind 0 5 0 / move 1 0 0 / land / exit
  3)  plotflight                            plots of the newest flight
Workspace: ~/Dev/dronecpp-ros2_ws   (open it in VS Code with:  code ~/Dev/dronecpp-ros2_ws)
EOF
