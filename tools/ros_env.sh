# Source this (don't run it) to get a ROS 2 Lyrical shell that can open Gazebo/RViz windows.
#   source tools/ros_env.sh
#
# Why the snap cleanup: VS Code installed as a snap exports GTK/GIO/locale paths pointing
# into /snap/code/... Every terminal it spawns inherits them, and Gazebo's GUI then loads
# the snap's core20 libpthread and crashes ("undefined symbol: __libc_pthread_init").
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR \
      GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH
if [[ -n ${XDG_DATA_DIRS:-} ]]; then
  XDG_DATA_DIRS=$(tr ':' '\n' <<<"$XDG_DATA_DIRS" | grep -v '/snap/code/' | paste -sd:)
  export XDG_DATA_DIRS
fi

# ROS setup.bash references unset variables, so source it before any `set -u`.
source /opt/ros/lyrical/setup.bash

# This repo is the colcon workspace; source its build if there is one (step 03+).
TB3_WS=${TB3_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}
if [[ -f $TB3_WS/install/setup.bash ]]; then
  source "$TB3_WS/install/setup.bash"
fi
