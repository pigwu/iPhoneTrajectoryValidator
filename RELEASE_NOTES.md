# iPhone Trajectory Validator v1.0.0

## English

- First standalone release of the iPhone Trajectory Validator.
- Validates iPhone ARKit trajectory logs against robot end-effector ground truth CSV files.
- Uses offline validation only: load two CSV files, generate result plots, inspect them in the GUI.
- Fixes ARKit scale at `1.0` to avoid scale overfitting.
- Includes click-to-preview result plots: 3D trajectory overlap, XYZ error curve, position error distribution, and motion error distribution.

## 中文

- iPhone Trajectory Validator 首个独立版本。
- 用机器人末端真值 CSV 验证 iPhone ARKit 轨迹 CSV 的准确性。
- 只保留离线验证流程：加载两份 CSV，自动生成结果图，并在 GUI 中查看。
- 固定 ARKit 尺度为 `1.0`，避免尺度拟合导致过拟合。
- 支持点击右侧缩略图，在中间放大查看 3D 轨迹重合图、XYZ 误差曲线、位置误差分布和运动误差分布。
