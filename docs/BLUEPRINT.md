# 组件、蓝图节点与状态

下面的内容移自重写前的 README。使用手册见文档站 https://astreox.github.io/uBotDocs/physics/。

## 组件与蓝图节点

### `uBot Physics Binding`

| 属性 | 说明 |
| --- | --- |
| `Backend` | 模拟这些物体的后端名（`UBotPhysics.Backend` 扩展点上注册的名字）。调用 `Bind` 时读取 |
| `Bodies` | 要交出的组件列表，只能是这个 Actor 自己的组件（`Component` 选择组件，`Body Name` 是它在后端模型里的名字，`None` 由后端自己决定）。留空：Actor 的根组件，必须是 Primitive 组件 |
| `Bind On Begin Play` | 默认开启。关闭后自己调用 `Bind`，例如等后端的模型准备好之后 |

| 节点 | 说明 |
| --- | --- |
| `Bind Physics Bodies` | 把所有物体交给后端，要么全部成功要么全部不交。已经绑定时直接返回 `true` |
| `Unbind Physics Bodies` | 取回物体并恢复 Chaos 模拟；同时清除失败的绑定状态和对应的运行时问题 |
| `Get Physics Binding Status` | `Unbound`、`Bound`、`BackendMissing`、`Rejected` |
| `Get Physics Binding Error` | 最近一次 `Bind` 失败的原因；其他情况为空 |
| `Get Bound Physics Components` | 已交出的组件，未绑定时为空 |

绑定组件实现了 UBotCore 的刚体状态源接口，只对自己已绑定的组件有效，见“刚体状态”。

### `uBot Physics Library`

类别 `uBot|Physics`：

| 节点 | 说明 |
| --- | --- |
| `Get uBot Physics Backend Names` | 已注册的后端名，按注册顺序 |
| `Get uBot Physics Backend Capabilities` | 一个后端的能力（`FUBotPhysicsCapabilities`）；没有这个后端时返回 `false` |
| `Get Body Physics Backend` | 模拟这个组件的后端名；`None` 表示 Chaos |
| `Get Dropped Physics Steps` | 这个世界里某后端因为一帧步数超限而丢弃的步数 |
| `Get Physics Scene Stats` | 这个世界里某后端现在看到的场景：静态物体数、运动学物体数、形状数，以及没能导出的组件数（`FUBotPhysicsSceneStats`）；后端没有求解器时返回 `false` |

`FUBotPhysicsCapabilities` 的字段只会追加：`bRigidBodies`、`bArticulations`、`bMaximalCoordinateJoints`、`bContacts`、`bStaticSceneGeometry`、`bKinematicForeignBodies`、`bAccelerations`、`bDeterministic`、`bDifferentiable`、`bSoftBodies`、`bCloth`、`bParticles`，默认全是 `false`，后端只声明自己真的支持的。其中 `bStaticSceneGeometry` 和 `bKinematicForeignBodies` 决定后端是否收到场景几何。能力在创建求解器时从工厂读取一次，之后不会再读：工厂的能力不能变，要换能力就注册一个新的工厂（相当于换掉这个后端）。

## 刚体状态

绑定组件实现 UBotCore 的 `IUBotBodyStateSource`（实验性）：对已绑定的组件，`ProvidesBodyState` 为真，`GetBodyState` 给出后端最近一步结束时的状态（`FUBotBodyState`，UE 世界轴，SI 单位）。提供一个组件同时也是**占有**它：别的绑定、别的状态源不能再驱动同一个组件。

- `TimestampSeconds` 是最近一步结束的 uBot 仿真时间，不晚于当前仿真时间。
- `Source` 是后端名（求解器没有设置时）。
- 后端第一步之前、或组件没有绑定时，`GetBodyState` 返回 `false`。
- `UBot::BodyState::GetNearestBodyState(Component, State)` 从组件起沿附着父级向上找最近的、被某个状态源提供的物体并返回它的状态；传感器挂在被绑定组件的子组件上时用它读真实速度。

C++ 里 `UUBotPhysicsSubsystem` 提供同样的查询（`FindBodyBackend`、`GetBodyState`）。

## 状态与蓝图接口

绑定失败时，`Bind` 向 uBot 运行状态报告一个问题，uBot Manager 会显示：

| 字段 | 值 |
| --- | --- |
| id | `physics.binding.<绑定组件的路径名>` |
| package | `UBotPhysics` |
| code | `physicsBindingFailed` |
| 级别 | Warning |
| 参数 | `actor`（Actor 名）、`backend`（后端名） |
| 消息 | 与 `Get Physics Binding Error` 相同 |

下一次成功的 `Bind`、`Unbind`，或绑定结束播放时问题被清除。id 带的是绑定组件自己的路径名（包含它所在的世界和 Actor），所以每个绑定组件各有一个 id：同一个 Actor 上有多个绑定组件，或不同世界里有同名的 Actor，都不会共用。后端在绑定期间被注销（见“绑定与所有权”第 6 条）时，受影响的绑定同样报告这个问题，消息说明后端已注销。
