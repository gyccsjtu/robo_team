#!/usr/bin/env bash
# Shared CUDA service, CPU-only perception clients; independent run directory.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export VISION_PYTHON="${VISION_PYTHON:-/root/robo_team_build/vision_env/bin/python}"
export SHARED_VISION_PYTHON="${SHARED_VISION_PYTHON:-/root/robo_team_build/vision_cuda_20261003/bin/python}"
export VISION_DEVICE=0
export PR_CLIENT_DEVICE=cpu
export PR_SHARED_INFER=1
export PR_FP16=0
export PR_SHARED_IMG_FMT=png
export CITY_PHYSICS_RATE="${CITY_PHYSICS_RATE:-20}"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
setup_file="${COORD_WS_SETUP:-/root/robo_team_build/codex_authority_v2/devel/setup.bash}"
set +u
source "$setup_file"
set -u
"$VISION_PYTHON" -c 'import cv2,numpy,rospy; from cv_bridge import CvBridge; from gazebo_msgs.srv import GetLinkState; from robocup_swarm.msg import SearchAssignment; from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo; print("CPU perception dependencies OK")'
"$SHARED_VISION_PYTHON" -c 'import torch,ultralytics,cv2; assert torch.cuda.is_available(), "CUDA unavailable"; print("Shared CUDA dependencies OK:",torch.__version__,torch.cuda.get_device_name(0))'
test -r "$repo_dir/weights/best_yolo11n_bino_v1.pt"
if [[ "${1:-}" == --check ]]; then
    echo 'Environment check passed. No simulator or inference service started.'
    exit 0
fi
exec bash "$repo_dir/run_city_swarm.sh" "$@"
