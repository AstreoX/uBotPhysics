# 开发

下面的内容移自重写前的 README。使用手册见文档站 https://astreox.github.io/uBotDocs/physics/。

## 在 uBot 系列中的位置

uBotPhysics 是 **Capability**（能力）层，依赖 `UBotCore ^0.1.3`（刚体状态源接口和多步调度从 0.1.3 起提供）。它不依赖 uBotSensor、uBotROS 和 uBotAerial，也不知道它们。

| Package | 关系 |
| --- | --- |
| UBotCore | 依赖。提供仿真时钟、固定步长调度器、刚体状态源接口、扩展点注册表和运行状态 |
| UBotSensor | 无依赖。传感器读到的是被写回的组件位姿，不关心它是谁算的；通过刚体状态源接口还能读到后端给出的真实速度 |
| UBotROS | 无依赖。ROS 只读组件位姿和 UBotCore 接口 |
| 物理后端插件 | 各自依赖本插件，在 `UBotPhysics.Backend` 注册工厂 |

## 模块

| 模块 | 类型 | 加载阶段 | 说明 |
| --- | --- | --- | --- |
| `UBotPhysics` | Runtime | Default | 后端接口、绑定组件、世界子系统、蓝图函数库 |

`"UBot"` 块声明的能力键（`Provides`）：`Physics.Backends`、`Physics.BodyBinding`、`Physics.SceneGeometry`；标签 `physics`。模块公有依赖 `Core`、`CoreUObject`、`Engine` 和 `UBotCore`，私有依赖 `ChaosCore`、`Chaos` 和 `PhysicsCore`（读取碰撞形状），日志类别只有 `LogUBotPhysics`。

## 测试

测试名都以 `UBotPhysics.` 开头，按类别分组：

- `Backends.ExtensionPoint`：模块声明了扩展点，测试后端能按名字找到、列出、报告能力，注册和注销，未知名字的处理。
- `Binding.*`：
  - `ClaimAndRelease`：绑定后组件变为运动学体，`FindBodyBackend`、`GetBoundComponents`、`ProvidesBodyState`，解绑恢复模拟并交还速度，`Bind On Begin Play`、结束播放和销毁组件时解绑；
  - `Conflicts`：同一个 Actor 上的第二个绑定、被别的状态源驱动、不是 Movable、根组件不是 Primitive 组件、引用不到组件、重复列出，全部以 `Rejected` 和错误说明拒绝；
  - `OwnActorOnly`：别的 Actor 的组件被拒绝（单独列出或夹在自己的组件后面都一样，什么都不会被取走）；绑定成功后，`GetNearestBodyState` 从被绑定组件的子组件（包括别的 Actor 附着在它下面的根组件）找到物体的状态；
  - `BackendMissing`：状态、运行时问题的字段，以及 `Unbind`、成功的 `Bind`、结束播放对它的清除；
  - `BackendUnregistered`：后端在绑定期间被注销，物体回到 Chaos 并带着最后的速度，绑定变为 `BackendMissing` 并报告运行时问题，求解器销毁、丢弃步数的计数保留，后端重新注册后再 `Bind` 创建新的求解器；
  - `BackendLostWhileBinding`：后端在 `AddBody`（已接受第二个物体之后）或 `CreateSolver` 里注销自己、或在 `AddBody` 里换成另一个工厂对象：绑定失败为 `BackendMissing`，已接受的物体都用 `RemoveBody` 取回，模拟开关恢复，没有留下求解器或所有权，运行时问题被报告，后端重新注册后可以再绑定；
  - `NestedSolverCreation`：后端的 `CreateSolver` 里触发同一个后端的另一次绑定：嵌套的绑定被拒绝（`Rejected`，错误说求解器还在创建，报告运行时问题，组件仍由 Chaos 模拟），外层的绑定成功，后端只创建了一个求解器，求解器只持有外层的物体；`CreateSolver` 返回后嵌套的绑定可以成功，两个物体各用自己的句柄回写，解绑后物体计数归零、空闲的求解器不再步进；
  - `BackendUnregisteredOffThread`：注册表在别的线程被修改时，子系统在那个线程上不碰组件和求解器，推迟到游戏线程处理；工厂被换成另一个对象时，下一次 `Bind` 先取回旧求解器的物体再用新工厂；
  - `ProblemIds`：不同世界、同一个 Actor 上的多个绑定组件，运行时问题的 id 各不相同，清除一个不影响另一个；
  - `Rollback`：求解器拒绝其中一个物体时，已交出的全部取回，模拟开关恢复，所有权不残留；
  - `SolverCreation`：后端没有给出求解器、步长不合法时的拒绝，以及求解器的复用。
- `Stepping.*`：
  - `FollowsSimClock`：不规则帧间隔下，步数等于仿真时间网格上经过的点数，步的结束时间落在网格上，最后一步不晚于仿真时间；
  - `DropsBeyondMax`：一帧步数超限时只执行上限的步数，其余计为丢弃，且不补跑；
  - `WritesBackPoses`：组件位置、旋转和 `ComponentVelocity` 跟随求解器，状态的时间戳和来源，`GetNearestBodyState` 从子组件找到物体，解绑后不再回写；
  - `WriteBackOrder`：三层附着的被绑定组件（按最深的在前交出），每一层都落在自己求解器的位姿上；
  - `WriteBackAcrossBackends`：父组件和子组件属于不同后端、子组件的后端先创建时，子组件同样不被父组件带偏；
  - `WriteBackEveryFrame`：求解器这一帧没有步进，被别的东西挪走的组件仍被写回求解器的位姿；
  - `WriteBackReentrancy`：被绑定组件的变换回调在写回途中解绑另一个被绑定组件、解绑自己、解绑后又重新绑定自己：被解绑的物体得不到过时的位姿，被解绑或换了句柄的物体得不到过时的速度，其余物体照常写回；
  - `WriteBackGeneration`：求解器把刚释放的句柄再发给新物体（测试后端的一个选项）时，被绑定组件的变换回调在写回途中把别的被绑定组件、或它自己解绑后又重新绑定（速度也换了）：重新绑定的物体不会被当成原来的持有关系，过时的位姿和速度都不写入，下一帧起按新物体写回；
  - `TickFunction`：Tick 函数在播放开始时注册，在 `TG_PrePhysics`，暂停时不运行，恢复后不补跑；
  - `IdleSolverRestarts`：没有物体的求解器不步进，重新绑定后时钟重新启动，不补跑空闲的时间。
- `Scene.*`：
  - `CollisionShapes`：`GetCollisionShapes` 对盒体、球体、胶囊体组件（带旋转和非均匀缩放，盒体还有镜像缩放）以及引擎的 `Cube`、`Cylinder` 静态网格（含镜像缩放）：类型、尺寸和变换（米），与测试自己用 `GetFinalScaled` 和逐轴缩放顶点算出的结果比较；带自己变换的凸包、太小的凸包和锥形胶囊（计入略过的形状）；实例化静态网格和蒙皮网格返回 `false`；三角网格走私有的转换函数，用合成的三角形检查缩放、单位，以及镜像缩放调换绕向（没有在测试里烹饪网格）；
  - `StaticAndKinematic`：一个静态盒、一个可移动盒和一个绑定的物体：第一帧两个盒以正确的类型、ID、米制变换和形状加入，可移动的盒在 `KinematicPoses` 里，物理材质来自组件；盒移动后下一次更新只有它的新位姿，静态盒不再发送；
  - `Capabilities`：两项能力都没有则从不调用 `UpdateScene`；只有静态则没有运动学物体；只有运动学则没有静态物体；没有物体的求解器不收更新，重新持有物体后补上期间的变化；
  - `OwnBodiesExcluded`：两个后端：A 绑定了一个带附着子盒的 Actor，A 的场景里没有这两个组件，B 的场景里有，且是运动学物体；A 解绑后下一次更新加入它们，ID 与 B 看到的相同，B 的 ID 不变；
  - `Changes`：销毁组件、关闭碰撞、给 Actor 或组件加 `UBotPhysics.NoScene`：`Removed`；重新打开碰撞、去掉标签：以新 ID 加入；改世界缩放或 Mobility：同一次更新里 `Removed` 加新 ID 的 `Added`；没有物理碰撞的组件从不发送；蒙皮网格和实例化静态网格计入略过的组件；
  - `UpdateBeforeSteps`：`UpdateScene` 在这一帧的 `Step` 之前，每帧最多一次，`SimTimeSeconds` 是这一帧的仿真时间，这一帧没有步进也调用；
  - `ReleasedBackend`：注销后端，求解器和它的场景视图随之消失；重新注册并绑定后，新求解器的第一次更新带整个场景；
  - `Stats`：`GetSceneStats` 和蓝图节点与求解器看到的场景及略过的组件数一致，两个后端各有各的统计。
- `Package.Descriptor`：`.uplugin` 的 `UBot` 块（层、对 UBotCore 的依赖、能力键、标签）。

测试在手动 tick 的临时 Game 世界里运行，用一个测试后端（每个物体匀速直线运动，能力可以配置，并记录收到的每次场景更新和它们累积出的场景），不需要 GPU，也不需要真实的物理后端。需要在启用了 UBotCore 和 UBotPhysics 的工程里用带编辑器上下文的 `UnrealEditor` 运行。编辑器内：`Tools` 菜单打开 Session Frontend 的 Automation 标签页，过滤 `UBotPhysics`。命令行（`UnrealEditor-Cmd` 和 `.uproject` 的路径按你的环境填写）：

```
UnrealEditor-Cmd <Project>.uproject -unattended -nop4 -nosplash -NullRHI "-ExecCmds=Automation RunTests UBotPhysics; Quit" "-TestExit=Automation Test Queue Empty"
```

把 `UBotPhysics` 换成更具体的前缀（如 `UBotPhysics.Binding`）可以只跑一类。UBotCore 自己的测试用 `UBotCore` 前缀。
