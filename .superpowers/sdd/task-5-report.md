# Task 5 Fix Report

## Fix: Removed unused import `_rpy_to_rot` from `calibration/calib_tools.py`

- **File**: `calibration/calib_tools.py`
- **Line 177**: Changed `from arm_skills import forward_kinematics, _rpy_to_rot` to `from arm_skills import forward_kinematics`
- **Reason**: `_rpy_to_rot` was imported but never used in the function. The function `_rotation_to_rpy()` is used instead.
- **Verified**: `from calibration.calib_tools import CHESSBOARD` imports successfully.

