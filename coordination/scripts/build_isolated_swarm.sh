#!/usr/bin/env bash
# Build only this repository's navigation and swarm packages in a separate tree.
set -euo pipefail
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
team_build_dir=${TEAM_BUILD_DIR:?Set TEAM_BUILD_DIR to an isolated absolute output directory}
case "$team_build_dir" in /*) ;; *) echo 'TEAM_BUILD_DIR must be absolute' >&2; exit 2;; esac
set +u
source /opt/ros/noetic/setup.bash
set -u
mkdir -p "$team_build_dir/src"
for package in robocup_navigation robocup_swarm; do
    target="$repo_root/coordination/src/$package"
    if [ -L "$team_build_dir/src/$package" ]; then
        [ "$(readlink -f "$team_build_dir/src/$package")" = "$(readlink -f "$target")" ] || exit 3
    elif [ -e "$team_build_dir/src/$package" ]; then
        echo "Refusing to replace existing $team_build_dir/src/$package" >&2; exit 3
    else
        ln -s "$target" "$team_build_dir/src/$package"
    fi
done
cd "$team_build_dir"
catkin_make -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3
echo "Built this repository; source $team_build_dir/devel/setup.bash"
