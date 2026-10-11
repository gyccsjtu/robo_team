# Pinned BoT-SORT source

Source: https://github.com/ultralytics/ultralytics/tree/69c95d1add13a232382e53b3aa5094ec0c7738db/ultralytics/trackers

Commit: `69c95d1add13a232382e53b3aa5094ec0c7738db`, source version 8.4.175. These tracker files are vendored independently of the detector package; the validated detector environment remains Ultralytics 8.3.0. The original AGPL-3.0 LICENSE and file notices are retained.

Local changes: byte_tracker.py imports LOGGER/xywh2ltwh from the installed Ultralytics utility package; matching.py fails clearly when lap is missing instead of automatically installing packages during flight; reid.py uses the no-argument smart_inference_mode decorator supported by the installed 8.3.0 utility API. Package __init__ files only expose this subset. ReID remains disabled; no new model is downloaded.

Files: basetrack.py, byte_tracker.py, bot_sort.py, utils/gmc.py, utils/matching.py, utils/kalman_filter.py, utils/stracks.py, utils/reid.py. The detector's utility API compatibility is checked in the isolated simulation environment. Camera timing, color isolation and output-evidence rules live in the project adapter image_mot.py.
