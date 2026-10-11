#!/usr/bin/env bash
# Build the selected snapshot's ROS1 messages in a new independent workspace.
set -eo pipefail
_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
_build="/root/robo_team_build/botsort_messages_20261011"
source /opt/ros/noetic/setup.bash
mkdir -p "$_build"
mkdir -p "$_build/src"
# Windows checkout stores the original catkin symlink as a text file. Restore
# only the new build workspace's link; never alter the checked-in source.
[ -e "$_build/src/CMakeLists.txt" ] || ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake "$_build/src/CMakeLists.txt"
for _package in robocup_navigation robocup_swarm; do
    [ -e "$_build/src/$_package" ] || ln -s "$_repo/coordination/src/$_package" "$_build/src/$_package"
done
catkin_make -C "$_build" --source "$_build/src" --build "$_build/build_isolated" \
    -DCATKIN_DEVEL_PREFIX="$_build/devel" -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    '-DCATKIN_WHITELIST_PACKAGES=robocup_navigation;robocup_swarm' -j4 -l4
