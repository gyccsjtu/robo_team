#!/bin/bash
V=/mnt/d/a/.robocup/robo_team/coordination/docs/validation/white_visibility_20261008
export PR_WEIGHTS=/mnt/d/a/.robocup/robo_team/weights/best_yolo11n_bino_v1.pt
export PR_PERSON_VERIFY_WEIGHTS=/mnt/d/a/.robocup/robo_team/weights/yolo11n_person.pt
PY=/root/robo_team_build/vision_cuda_20261003/bin/python
[ -x "$PY" ] || PY=python3
"$PY" "$V/white_frame_replay.py" \
  --round /root/robocup_runs/codex_legacy_chain_v140_20261008/round_1_seed_8159 \
  --audit-json "$V/white_visibility_v140_codex.json" \
  --json-out "$V/white_frame_replay_v140.json"