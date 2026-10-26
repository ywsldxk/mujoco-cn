<div align="center">

[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.17.0-39a9db?style=flat-square)](https://github.com/google-deepmind/mujoco)
[![Python](https://img.shields.io/badge/Python-3.11%2B-3776ab?style=flat-square)](https://www.python.org/)
[![Windows](https://img.shields.io/badge/Platform-Windows-0078d4?style=flat-square)](https://www.microsoft.com/windows)
[![License](https://img.shields.io/badge/License-Apache--2.0-green?style=flat-square)](LICENSE)
[![GitHub stars](https://img.shields.io/github/stars/ywsldxk/mujoco-cn?style=flat-square)](https://github.com/ywsldxk/mujoco-cn)
[![Hugging Face](https://img.shields.io/badge/Hugging%20Face-mujoco--cn-yellow?style=flat-square)](https://huggingface.co/ywsldxk1/mujoco-cn)

# mujoco-cn

**中文菜单的 MuJoCo 桌面操作仿真器**

SO-101 键盘遥操作 · OBJ / STL / MJCF 多物体导入 · 可扩展场景

[简体中文](README.md) · [English](README.en.md)

</div>

---

## 功能

- 中文左右折叠菜单，场景视图保持在中央。
- 用 W/S、A/D、I/K、J/L、Ctrl 和夹爪按键控制 SO-101。
- 在“文件 → 导入对象”里一次选择多个 OBJ、STL 或独立 MJCF XML。
- 查看关节角度、拖动控制滑杆、切换视觉/碰撞几何组和接触可视化。
- 使用官方 Menagerie 的 The Robot Studio SO101 模型作为默认演示机器人。

## Windows 快速开始

需要 Python 3.11 或更新版本。在 PowerShell 中运行：

```powershell
.\scripts\setup.ps1
.\scripts\run.ps1
```

`.venv` 和 Menagerie 模型缓存都放在项目目录中。安装脚本不会把 MuJoCo 或 Python 包安装到 C 盘。

## 键盘控制

| 按键 | 控制 |
| --- | --- |
| W / S | 肩部升降 |
| A / D | 肩部左右转 |
| I / K | 肘部 |
| J / L | 腕部旋转 |
| 左 / 右 Ctrl | 腕部俯仰 |
| C / B | 合爪 |
| V / N | 张爪 |
| 空格 | 切换夹爪 |
| F2 | 重置演示瓶子 |
| Esc | 退出 |

鼠标左键拖动旋转视角，滚轮缩放。右侧“控制”菜单提供关节滑杆。

## 导入物体

展开左侧“文件”菜单，点“导入对象（可多选）”，然后选一个或多个支持的文件：

- **OBJ / STL**：自动包装为可移动物体。导入时需要输入网格单位缩放；毫米模型通常填 `0.001`，米制模型填 `1`。碰撞使用网格凸包，复杂凹形物体建议导入带简化碰撞几何的 MJCF。
- **MJCF XML**：用于独立物体模型。不要导入含地板、灯光等环境内容的整套场景 XML。

导入后程序会重新编译场景，并保留机械臂和演示瓶子的当前姿态。新物体排列在机械臂前方。对象格式适配代码位于 [`object_import.py`](object_import.py)，可在此扩展更多格式。

## 依赖与上游许可

本项目代码采用 Apache-2.0。MuJoCo 本身和 Menagerie 的 `robotstudio_so101` 模型也采用 Apache-2.0；模型由 `mujoco-menagerie` 在运行时下载，不随本仓库分发。[MuJoCo](https://github.com/google-deepmind/mujoco) · [SO101 Menagerie 模型](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101)

mujoco-cn 是独立社区项目，不是 Google DeepMind 官方产品。第三方声明见 [NOTICE](NOTICE)。
