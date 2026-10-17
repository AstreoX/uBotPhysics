<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/brand/ubot-mark-light.svg">
    <source media="(prefers-color-scheme: light)" srcset="docs/brand/ubot-mark-dark.svg">
    <img alt="uBot" src="docs/brand/ubot-mark-dark.svg" height="88">
  </picture>
</p>
<h1 align="center">uBot Physics</h1>
<p align="center"><b>同一个关卡里并存多个物理后端，Chaos 仍是默认。</b></p>
<p align="center">
  <a href="LICENSE"><img alt="license" src="https://img.shields.io/github/license/AstreoX/uBotPhysics"></a>
  <img alt="Unreal Engine 5.5" src="https://img.shields.io/badge/Unreal%20Engine-5.5-313131">
</p>
<p align="center">
  <a href="https://astreox.github.io/uBotDocs/physics/">文档</a> ·
  <a href="https://github.com/AstreoX/uBotPhysics/issues">问题反馈</a>
</p>

<div align="center">

[English](README.md) | **简体中文**

</div>

## 简介

uBot Physics 是 uBot 系列中的多物理后端插件，适用于 Unreal Engine 5.5。同一个关卡里，被绑定的物体由你指定的物理后端计算：后端按 uBot 仿真时间以自己的固定步长推进，每帧把位姿写回对应的 UE 组件。没有绑定的物体仍由 UE 自带的 Chaos 模拟。

- **后端并存**：按名字把组件绑定到某个后端，其余物体仍归 Chaos。
- **蓝图优先**：给 Actor 加上 `uBot Physics Binding` 组件，选择后端，播放时自动绑定。
- **后端能看到场景**：声明了相应能力的后端，会把关卡里其余的碰撞体当作静态物体和运动学物体收到。
- **可复现**：后端的步序落在仿真时间网格上；暂停时不步进，恢复后不补跑。
- **后端是独立的插件**：本插件只提供后端接口和绑定规则，自己不带任何物理引擎。

## 状态

**0.1.0（Beta），尚未发布。** 后端接口（`IUBotPhysicsBackendFactory`、`IUBotPhysicsSolver`）是实验性的，小版本之间可能变化。

## 依赖

- Unreal Engine 5.5 与 C++ 工程。插件以源码形式分发。
- [uBotCore](https://github.com/AstreoX/uBotCore) `^0.1.3`。0.1.3 同样尚未发布。
- 至少一个物理后端插件，用来接收被绑定的物体。没有后端时，绑定会失败并报告 `BackendMissing`，物体仍由 Chaos 模拟。

## 文档

使用手册发布在[文档站](https://astreox.github.io/uBotDocs/physics/)。文档站和仓库 `docs/` 下的 Markdown 文件均以中文撰写。

| 我想… | 去这里 |
| --- | --- |
| 了解绑定、所有权和每帧时序 | [docs/CONCEPTS.md](docs/CONCEPTS.md) |
| 用 C++ 编写物理后端 | [docs/BACKENDS.md](docs/BACKENDS.md) |
| 了解后端收到的场景几何 | [docs/SCENE.md](docs/SCENE.md) |
| 查阅组件、蓝图节点和绑定状态 | [docs/BLUEPRINT.md](docs/BLUEPRINT.md) |
| 了解模块划分、描述块和测试方法 | [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) |

## uBot 系列

| 仓库 | 说明 |
| --- | --- |
| [uBotCore](https://github.com/AstreoX/uBotCore) | 仿真时钟、帧头、共享接口、插件管理 |
| [uBotSensor](https://github.com/AstreoX/uBotSensor) | 位姿、IMU、里程计、2D/3D 激光雷达、RGB 与深度相机 |
| [uBotROS](https://github.com/AstreoX/uBotROS) | 通过 TCP 桥接入 ROS 2：传感器、TF、/clock、cmd_vel |
| [uBotAerial](https://github.com/AstreoX/uBotAerial) | 多旋翼无人机：简化飞行、刚体动力学、风 |
| **[uBotPhysics](https://github.com/AstreoX/uBotPhysics)** | **多物理后端与 Chaos 并存** |
| [uBotManager](https://github.com/AstreoX/uBotManager) | 在 UE 工程里安装、更新、构建 uBot 包的桌面程序和命令行 |
| [uBotDocs](https://github.com/AstreoX/uBotDocs) | 文档站：https://astreox.github.io/uBotDocs/ |

## 开发

- **测试**：自动化测试以 `UBotPhysics.` 开头。打开 `Tools > Session Frontend`，切换到 Automation 标签页并过滤 `UBotPhysics`；命令行运行方式见 [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)。
- **问题反馈**：在 [GitHub Issues](https://github.com/AstreoX/uBotPhysics/issues) 提交，并注明 uBot Physics 版本和编辑器日志中相关的 `LogUBotPhysics` 记录。

## 许可证

[Apache License 2.0](LICENSE)，另见 [NOTICE](NOTICE)。Copyright 2026 AstreoX。允许使用、修改和再分发（包括商业用途）；再分发时须保留 `LICENSE` 与 `NOTICE` 文件，并标明修改过的文件。
