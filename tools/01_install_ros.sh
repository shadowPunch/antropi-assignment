#!/usr/bin/env bash
# Step 01: install ROS 2 Lyrical desktop + Nav2, TurtleBot3 sim, SLAM and teleop on Ubuntu 26.04.
# Follows the official deb install guide. Run as your normal user:
#   bash tools/01_install_ros.sh
# It asks for root once: via sudo in a terminal, or a graphical polkit dialog otherwise.
set -euo pipefail

ROS_DISTRO=lyrical

install_packages() {  # runs as root
  export DEBIAN_FRONTEND=noninteractive

  # Repositories: Ubuntu universe + ROS 2 apt source.
  apt-get update
  apt-get install -y software-properties-common curl
  add-apt-repository -y universe

  if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    local version codename deb
    version=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
    deb=$(mktemp --suffix=.deb)
    curl -fL -o "$deb" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.${codename}_all.deb"
    dpkg -i "$deb"
    rm -f "$deb"
  fi

  # ROS desktop, dev tools (colcon etc.), and the navigation stack.
  apt-get update
  apt-get upgrade -y   # recommended by the ROS guide: debs are built against an up-to-date system
  apt-get install -y \
    ros-dev-tools \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    "ros-${ROS_DISTRO}-nav2-minimal-tb*" \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-nav2-simple-commander
}

if [[ ${1:-} == --as-root ]]; then
  install_packages
  exit 0
fi

if [[ $EUID -eq 0 ]]; then
  echo "Run as your normal user, not root (the script edits your ~/.bashrc)." >&2
  exit 1
fi

self=$(realpath "$0")
if [[ -t 0 ]]; then
  sudo bash "$self" --as-root
else
  pkexec bash "$self" --as-root   # no terminal: GNOME shows a password dialog
fi

# Source ROS in every new terminal (idempotent).
line="source /opt/ros/${ROS_DISTRO}/setup.bash"
grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc

echo "Done. Open a new terminal (or: $line) and run tools/01_check_install.sh"
