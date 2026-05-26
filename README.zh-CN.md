# iPhone Trajectory Validator

<div align="center">

![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)
![Platform: Windows%20%7C%20macOS-blue](https://img.shields.io/badge/platform-Windows%20%7C%20macOS-blue.svg)
![Mode: Offline](https://img.shields.io/badge/mode-offline%20validation-orange.svg)

**用于验证 iPhone ARKit 轨迹精度的离线工具。**

**语言 / Language**: [English](README.md) | [简体中文](README.zh-CN.md)

![iPhone Trajectory Validator hero](assets/hero-banner.png)

</div>

---

## 这是什么

iPhone Trajectory Validator 用来验证 iPhone ARKit 记录的轨迹，在手机自己的坐标系里是否可信。

机械臂末端轨迹只作为真值参考，用来验证手机轨迹。除非你的正式业务需要把手机轨迹转换到机器人 base 坐标系，否则这些机器人相关变换不需要复用。

当前工具采用保守验证策略：

- ARKit 尺度固定为 `1.0`。
- 不拟合 scale，也不优化 scale。
- 机械臂只是验证用的真值尺子，不是运行时依赖。
- GUI 只保留离线流程：加载 CSV，生成结果图，查看结果。

## 演示

![点击预览演示](assets/validator-demo.gif)

## 结果查看

打开后，中间默认显示最终 3D 轨迹重合图。

![3D trajectory overlap](assets/screenshots/trajectory_overlap_3d.png)

点击右侧任意结果缩略图，会在中间大图区域放大查看。

<table>
  <tr>
    <td width="50%"><img src="assets/screenshots/axis_error_curves.png" alt="XYZ error curve"></td>
    <td width="50%"><img src="assets/screenshots/absolute_error_histogram.png" alt="Position error distribution"></td>
  </tr>
  <tr>
    <td align="center">XYZ 误差曲线</td>
    <td align="center">位置误差分布</td>
  </tr>
  <tr>
    <td colspan="2"><img src="assets/screenshots/relative_error_histogram.png" alt="Motion error distribution"></td>
  </tr>
  <tr>
    <td colspan="2" align="center">相对运动误差分布</td>
  </tr>
</table>

## 基础流程

```text
iPhone ARKit CSV
        +
机器人末端真值 CSV
        |
        v
时间同步
        |
        v
固定 scale = 1.0
        |
        v
仅用于评估的手眼关系与 session 对齐
        |
        v
误差指标 + 结果图
```

## 输出内容

每次验证会在 `offline_calibration_output/` 下生成一个带时间戳的结果目录，包含：

- `trajectory_overlap_3d.png`：最终 3D 轨迹重合图
- `axis_error_curves.png`：X/Y/Z 三轴误差曲线
- `absolute_error_histogram.png`：位置误差分布
- `relative_error_histogram.png`：相对运动误差分布
- `offline_calibration_result.json`：数值指标和评估时用到的变换

GUI 中最重要的数值是：

- 匹配样本数量
- 平均位置误差
- RMSE 位置误差
- 最大位置误差
- 时间偏移

## 安装

```bash
pip install -r requirements.txt
```

## 运行 GUI

Windows:

```powershell
.\run_validator_windows.ps1
```

macOS / Linux:

```bash
chmod +x ./run_validator_mac.sh
./run_validator_mac.sh
```

也可以直接运行：

```bash
python iphone_trajectory_validator.py
```

## 命令行运行

```bash
python iphone_trajectory_validator.py --mode offline \
  --arkit-csv path/to/iphone_pose.csv \
  --sensor-csv path/to/robot_end_effector.csv \
  --output-dir offline_calibration_output
```

## 数据要求

iPhone ARKit CSV 需要包含位置和四元数姿态字段。

机器人 CSV 需要包含末端位置和姿态字段，或当前 loader 支持解析的等价位姿字段。

两份日志需要覆盖同一段运动，并且运动幅度要足够，才能稳定完成对齐和误差评估。

## 如何理解结果

如果最终 3D 重合图贴合得很好，并且误差指标很小，可以得出实际结论：

> 在当前采集设置下，iPhone ARKit 轨迹在手机自身坐标系里是可信的。

如果正式业务只使用手机自身坐标系，就不需要复用机器人相关变换。机械臂和这些矩阵只是验证阶段的参考。

## 为什么固定 scale = 1.0

实验结果表明，在当前验证设置下 ARKit 自身尺度已经足够一致。拟合 scale 可能让某一组样本误差更小，但存在过拟合风险。因此本工具固定 `scale_factor = 1.0`，不做尺度拟合。

## 许可证

MIT
