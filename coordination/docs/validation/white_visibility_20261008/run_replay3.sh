#!/bin/bash
V=/mnt/d/a/.robocup/robo_team/coordination/docs/validation/white_visibility_20261008
R=/root/robocup_runs/codex_legacy_chain_v140_20261008/round_1_seed_8159
PY=/root/robo_team_build/vision_cuda_20261003/bin/python
[ -x "$PY" ] || PY=python3
export PR_WEIGHTS=/mnt/d/a/.robocup/robo_team/weights/best_yolo11n_bino_v1.pt
export PR_PERSON_VERIFY_WEIGHTS=/mnt/d/a/.robocup/robo_team/weights/yolo11n_person.pt
export PYTHONPATH=/mnt/d/a/.robocup/robo_team/perception
"$PY" "$V/white_replay3.py" \
  --round "$R" \
  --audit-json "$V/white_audit3_v140.json" \
  --json-out "$V/white_replay3_v140.json" \
  --annot-dir "$V/annotated"