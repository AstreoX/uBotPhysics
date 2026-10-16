<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/brand/ubot-mark-light.svg">
    <source media="(prefers-color-scheme: light)" srcset="docs/brand/ubot-mark-dark.svg">
    <img alt="uBot" src="docs/brand/ubot-mark-dark.svg" height="88">
  </picture>
</p>
<h1 align="center">uBot Physics</h1>
<p align="center"><b>Multiple physics backends in one Unreal Engine world, with Chaos as the default.</b></p>
<p align="center">
  <a href="LICENSE"><img alt="license" src="https://img.shields.io/github/license/AstreoX/uBotPhysics"></a>
  <img alt="Unreal Engine 5.5" src="https://img.shields.io/badge/Unreal%20Engine-5.5-313131">
</p>
<p align="center">
  <a href="https://astreox.github.io/uBotDocs/physics/">Documentation</a> ·
  <a href="https://github.com/AstreoX/uBotPhysics/issues">Issues</a>
</p>

<div align="center">

**English** | [简体中文](README.zh-CN.md)

</div>

## What it is

uBot Physics is the multi-backend physics plugin of the uBot family for Unreal Engine 5.5. In one level, a bound body is simulated by the physics backend you pick: the backend steps on the uBot simulation clock with its own fixed step, and writes its poses back to the UE component every frame. Bodies that are not bound stay with Chaos, the engine's built-in physics.

- **Backends side by side.** Bind a component to a backend by name; everything else stays with Chaos.
- **Blueprint first.** Add the `uBot Physics Binding` component to an actor, choose a backend, and it binds on Play.
- **Backends can see the scene.** A backend that declares the capability receives the other colliders in the level as static and kinematic bodies.
- **Reproducible.** Backend steps lie on the simulation time grid. Pausing stops stepping, and resuming does not catch up.
- **Backends are separate plugins.** uBot Physics provides the backend interfaces and the binding rules. It contains no physics engine of its own.

## Status

**0.1.0 (Beta), not released yet.** The backend interfaces (`IUBotPhysicsBackendFactory`, `IUBotPhysicsSolver`) are experimental and may change between minor versions.

## Dependencies

- Unreal Engine 5.5 and a C++ project. The plugin is distributed as source.
- [uBotCore](https://github.com/AstreoX/uBotCore) `^0.1.3`. Version 0.1.3 is not released yet either.
- At least one physics backend plugin to bind bodies to. Without one, a binding fails with `BackendMissing` and the bodies stay with Chaos.

## Documentation

The manual is published on the [documentation site](https://astreox.github.io/uBotDocs/physics/). The site and the Markdown files in `docs/` are written in Chinese.

| I want to ... | Go to |
| --- | --- |
| Understand binding, ownership and per-frame timing | [docs/CONCEPTS.md](docs/CONCEPTS.md) |
| Write a physics backend in C++ | [docs/BACKENDS.md](docs/BACKENDS.md) |
| See what scene geometry a backend receives | [docs/SCENE.md](docs/SCENE.md) |
| Look up the component, Blueprint nodes and binding status | [docs/BLUEPRINT.md](docs/BLUEPRINT.md) |
| Find the module layout, the package descriptor and the test instructions | [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) |

## uBot family

| Repository | What it is |
| --- | --- |
| [uBotCore](https://github.com/AstreoX/uBotCore) | Simulation clock, frame header, shared interfaces, package management |
| [uBotSensor](https://github.com/AstreoX/uBotSensor) | Pose, IMU, odometry, 2D/3D lidar, RGB and depth cameras |
| [uBotROS](https://github.com/AstreoX/uBotROS) | ROS 2 over a TCP bridge: sensors, TF, /clock, cmd_vel |
| [uBotAerial](https://github.com/AstreoX/uBotAerial) | Multirotor drones: simplified flight, rigid-body dynamics, wind |
| **[uBotPhysics](https://github.com/AstreoX/uBotPhysics)** | **Physics backends side by side with Chaos** |
| [uBotManager](https://github.com/AstreoX/uBotManager) | Desktop app and CLI to install, update and build uBot packages |
| [uBotDocs](https://github.com/AstreoX/uBotDocs) | Documentation site: https://astreox.github.io/uBotDocs/ |

## Development

- **Test:** the Automation tests are named `UBotPhysics.*`. Open `Tools > Session Frontend`, select the Automation tab and filter by `UBotPhysics`. The command-line invocation is in [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md).
- **Issues:** report them on [GitHub Issues](https://github.com/AstreoX/uBotPhysics/issues), stating the uBot Physics version and the relevant `LogUBotPhysics` lines from the editor log.

## License

[Apache License 2.0](LICENSE), see also [NOTICE](NOTICE). Copyright 2026 AstreoX. The software may be used, modified and redistributed, including commercially, provided that the `LICENSE` and `NOTICE` files are kept and modified files are marked.
