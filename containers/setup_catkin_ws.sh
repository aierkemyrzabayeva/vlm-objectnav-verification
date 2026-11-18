#!/bin/bash
# Creates $VLMNAV_BASE/vlm_nav (a clone of the original system's repository at the commit used in the paper) and the sources of the catkin workspace:
#   - the packages multi_robot_scenario, my_robot_navigation, velodyne_simulator come from the original repository (catkin_ws/src/ at that commit)
#   - realsense2_description and realsense_gazebo_plugin (issaiass) are cloned at the pinned commits; our edit of the colour camera is applied as a patch
# Needs git and internet access. Afterwards: copy the files of code/ to $VLMNAV_BASE/vlm_nav/ours and run build_ws.sh.
set -eu
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
HERE=$(cd "$(dirname "$0")" && pwd)
ORIG=https://github.com/Yernar-SDU/Yernar-SDU-VLM_project-VLM_DRL-based-autonomous-robot-ObjectNav.git
ORIG_COMMIT=467a6ad441fc877eca0429a6a590f865590f0e59
mkdir -p "$BASE"
[ -d "$BASE/vlm_nav/.git" ] || git clone "$ORIG" "$BASE/vlm_nav"
git -C "$BASE/vlm_nav" checkout "$ORIG_COMMIT"
SRC=$BASE/vlm_nav/catkin_ws/src
for p in multi_robot_scenario my_robot_navigation velodyne_simulator; do
  [ -d "$SRC/$p" ] || { echo "missing $SRC/$p in the original repository at $ORIG_COMMIT"; exit 1; }
done
[ -e "$SRC/CMakeLists.txt" ] || echo "NOTE: $SRC/CMakeLists.txt is missing; create it with: cd $SRC && catkin_init_workspace (inside the ROS container)"

clone_at() {  # url dir commit
  [ -d "$SRC/$2/.git" ] || git clone "$1" "$SRC/$2"
  git -C "$SRC/$2" checkout "$3"
}
clone_at https://github.com/issaiass/realsense2_description.git realsense2_description d54b5e6503d3e33611df8c7ab152d442bb2b2841
clone_at https://github.com/issaiass/realsense_gazebo_plugin.git realsense_gazebo_plugin a64ea263754bd1e923f26a3290ee4e86e6aa291a

PATCH=$HERE/patches/realsense2_description_d435_color.patch
if git -C "$SRC/realsense2_description" apply --check "$PATCH" 2>/dev/null; then
  git -C "$SRC/realsense2_description" apply "$PATCH"; echo "camera patch applied (colour stream 1280x720, 85.2 deg)"
elif git -C "$SRC/realsense2_description" apply --check -R "$PATCH" 2>/dev/null; then
  echo "camera patch already applied"
else
  echo "the camera patch does not apply; check the commit of realsense2_description"; exit 1
fi
echo "Done. Next: mkdir -p $BASE/vlm_nav/ours && cp code/* $BASE/vlm_nav/ours/ ; then run build_ws.sh"
