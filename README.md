<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/brand/ubot-mark-light.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/brand/ubot-mark-dark.svg">
  <img alt="uBot" src="docs/brand/ubot-mark-dark.svg" height="96">
</picture>

# uBot Physics

## 简介

uBotPhysics 是 uBot 家族的多物理后端插件，为 UE 5.5 提供一种做法：**同一个关卡里，不同的物体由不同的物理后端计算**。UE 自带的 Chaos 仍是默认，没有被绑定的物体都归它；被绑定的物体交给指定的后端，后端按 uBot 仿真时间以自己的固定步长推进，每帧把结果写回对应的 UE 组件。它建立在 [uBotCore](https://github.com/AstreoX/uBotCore) 之上。

- **后端是独立的插件。** 本插件只提供后端接口（`IUBotPhysicsBackendFactory`、`IUBotPhysicsSolver`）、扩展点 `UBotPhysics.Backend`、绑定组件、所有权规则、按仿真时钟步进和位姿回写。它自己不带任何物理引擎；后端插件在扩展点注册一个工厂，名字就是绑定组件里 `Backend` 填的名字。
- **蓝图优先。** 绑定组件 `uBot Physics Binding` 加在 Actor 上，选后端，播放时自动绑定；查询和诊断也有蓝图节点。
- **UE 里的组件不变。** 被绑定的组件还是原来的网格或碰撞体，只是不再由 Chaos 模拟；传感器、射线检测、动画照常使用它。
- **可复现。** 后端的步序是仿真时间网格上的整数，不累加帧间隔；暂停时不步进，恢复后不补跑。
- **后端接口是实验性的。** 在只有少数实现的阶段，接口在小版本之间可能变化。

## 在 uBot 家族中的位置

uBotPhysics 是 **Capability**（能力）层，依赖 `UBotCore ^0.1.3`（刚体状态源接口和多步调度从 0.1.3 起提供）。它不依赖 uBotSensor、uBotROS 和 uBotAerial，也不知道它们。

| Package | 关系 |
| --- | --- |
| UBotCore | 依赖。提供仿真时钟、固定步长调度器、刚体状态源接口、扩展点注册表和运行状态 |
| UBotSensor | 无依赖。传感器读到的是被写回的组件位姿，不关心它是谁算的；通过刚体状态源接口还能读到后端给出的真实速度 |
| UBotROS | 无依赖。ROS 只读组件位姿和 UBotCore 接口 |
| 物理后端插件 | 各自依赖本插件，在 `UBotPhysics.Backend` 注册工厂 |

| 模块 | 类型 | 加载阶段 | 说明 |
| --- | --- | --- | --- |
| `UBotPhysics` | Runtime | Default | 后端接口、绑定组件、世界子系统、蓝图函数库 |

`"UBot"` 块声明的能力键（`Provides`）：`Physics.Backends`、`Physics.BodyBinding`；标签 `physics`。模块依赖 `Core`、`CoreUObject`、`Engine` 和 `UBotCore`，日志类别只有 `LogUBotPhysics`。

## 安装

面向 UE 5.5。插件以源码形式分发，工程需要能编译 C++。先安装 uBotCore（0.1.3 或更新）。

1. 把仓库克隆到 uBotCore 旁边，即 `<Project>/Plugins/uBot/uBotPhysics`：

   ```powershell
   git clone https://github.com/AstreoX/uBotPhysics.git <Project>/Plugins/uBot/uBotPhysics
   ```

   也可以用 [uBot Manager](https://github.com/AstreoX/uBotManager) 安装，它会先装好缺失的依赖；命令行等价于 `UnrealEditor-Cmd.exe <Project>.uproject -run=UBotPackage -Install=UBotPhysics`。

2. 启用插件：在 `Edit > Plugins` 中搜索 uBot Physics（分类 uBot）并勾选，或在 `.uproject` 中加入

   ```json
   "Plugins": [
       { "Name": "UBotCore", "Enabled": true },
       { "Name": "UBotPhysics", "Enabled": true }
   ]
   ```

3. 重启编辑器。组件在 Add Component 菜单的 **uBot** 分类下，显示名 `uBot Physics Binding`。

4. 安装并启用至少一个物理后端插件。没有后端时，绑定会失败并报告 `BackendMissing`，物体仍由 Chaos 模拟。

C++ 工程使用时，在自己模块的 `Build.cs` 中加入 `"UBotCore"` 和 `"UBotPhysics"`。

## 快速开始

1. 给要交给后端的 Actor 加 `uBot Physics Binding` 组件。
2. 在 Details 里把 `Backend` 设为后端的名字。可用的名字见 `Get uBot Physics Backend Names`。
3. `Bodies` 留空表示用 Actor 的根组件；根组件必须是 Primitive 组件（网格、盒体、胶囊体等），Mobility 必须是 `Movable`。要交出多个组件，在 `Bodies` 里逐个选择组件，并按后端的要求填 `Body Name`；这些组件必须属于绑定组件所在的这个 Actor，别的 Actor 的组件会被拒绝（要交出别的 Actor，给它自己加一个绑定组件）。
4. 点「Play」。`Bind On Begin Play` 默认开启，播放开始时组件调用 `Bind`；绑定成功后，这个组件由后端模拟，Chaos 不再模拟它。

绑定失败时不会有任何一个组件被交出。`Get Physics Binding Status` 和 `Get Physics Binding Error` 说明原因，失败还会作为运行时问题显示在 uBot Manager 里，见“状态与蓝图接口”。

## 概念

### 后端

**后端**是一个在扩展点 `UBotPhysics.Backend` 注册的工厂（`IUBotPhysicsBackendFactory`）。工厂的扩展名就是后端名；它声明后端的能力（`FUBotPhysicsCapabilities`），并为每个世界创建一个**求解器**（`IUBotPhysicsSolver`）。求解器只在游戏线程上使用。

一个世界里，每个后端最多有一个求解器，在第一次有物体绑定到它时创建，随世界一起释放。后端自带的模型（例如它自己的场景文件）由后端插件管理；本插件不规定模型格式，也不把 UE 的场景几何交给后端。

### 绑定与所有权

**绑定**把一个或几个 UE 组件交给某个后端。规则：

1. **一个组件最多属于一个后端。** 没有被绑定的组件归 Chaos。
2. **已经有别的状态源在驱动的组件不能绑定。** 例如被载具自己的积分器驱动、并实现了刚体状态源接口的机体。
3. **全部成功，或全部回滚。** 绑定按下面的顺序检查，任何一步失败都不会留下半截结果：
   1. 后端的工厂已注册，否则状态为 `BackendMissing`；
   2. 每个组件都能解析到同一个世界里的 Primitive 组件，属于绑定组件所在的 Actor，Mobility 是 `Movable`，没有被别的绑定占用，也没有别的刚体状态源在驱动它，没有重复列出，否则状态为 `Rejected`，错误说明是哪个组件、为什么。只交出自己 Actor 的组件，是因为刚体状态源按“组件所属的 Actor 和这个 Actor 的组件”查找，绑定组件在同一个 Actor 上才能被传感器等找到；
   3. 后端为这个世界创建求解器，且它的固定步长在 0.0001 s 到 100 s 之间，否则状态为 `Rejected`。后端的 `CreateSolver` 正在运行时，对同一个后端的另一次 `Bind` 同样是 `Rejected`，错误说明该后端的求解器还在创建；
   4. 逐个组件交给求解器：先把组件的物理模拟关掉，再调用求解器的 `AddBody`；求解器拒绝其中一个时，已交出的全部取回，模拟开关恢复原样，状态为 `Rejected`。

   创建求解器和 `AddBody` 都是后端自己的代码，它们可能在运行中途注销或换掉自己的工厂。子系统在 `CreateSolver` 返回后、每个 `AddBody` 返回后都会检查：注册的工厂还是不是刚才用的那个，子系统里这个后端的求解器是不是还是这一个。有一项变了，就把已经接受的物体（包括刚返回的那个）按相反的顺序取回，对仍然存在的求解器调用 `RemoveBody`，模拟开关恢复原样，绑定失败，状态为 `BackendMissing`，错误说明后端在交出物体时被注销或换掉了，并报告运行时问题。这种情况下没有任何物体留在后端里，后端重新注册后再调用 `Bind` 即可。

   `CreateSolver` 里也可能触发同一个后端的另一次绑定（后端自己的代码调用了 `Bind`）。这时求解器还没有创建完，子系统拒绝这次嵌套的绑定：状态 `Rejected`，错误说明该后端的求解器还在创建，并报告运行时问题；外层的绑定不受影响。外层的 `CreateSolver` 返回后，子系统只在这个后端还没有求解器时才采用它；如果槽位里已经有了求解器，就丢弃刚创建的这个，用槽位里的，已经交出的物体的计数不会被重置。嵌套的绑定在 `CreateSolver` 返回后重新调用 `Bind` 即可。这种重入不受支持，见“给后端作者”。
4. **解绑恢复原状。** 原来由 Chaos 模拟的组件重新开启模拟，并带上后端最后一步的线速度和角速度；原来就不模拟的组件保持不模拟。
5. 绑定组件结束播放、被销毁或世界结束时自动解绑。
6. **后端被注销时，物体回到 Chaos。** 后端注销自己的工厂（后端插件卸载，或换成另一个同名的工厂对象）时，子系统把这个后端的所有物体按解绑的方式取回（恢复模拟，带上最后一步的速度），这些绑定的状态变为 `BackendMissing`，错误说明后端已注销，并报告运行时问题；求解器随即销毁。后端重新注册后，再调用 `Bind` 会用新注册的工厂创建新的求解器。已经丢弃的步数计数保留。

被绑定的组件在 Chaos 里是**运动学体**：模拟关闭，碰撞设置不变。所以射线检测、传感器和 Chaos 的接触仍然能看到它，它也能推动 Chaos 里的动态物体。

### 每帧时序

后端的步进和位姿回写发生在每帧的 `TG_PrePhysics`，由世界子系统 `UUBotPhysicsSubsystem` 的 Tick 函数完成（暂停时不运行）：

```
UWorld::Tick 推进仿真时间
TG_PrePhysics     UBotPhysics：每个求解器按自己的步长补到不超过当前仿真时间；所有求解器都步进完之后，把所有被绑定物体的位姿写回运动学组件
TG_DuringPhysics  Chaos 模拟其余的物体；被绑定的组件作为运动学体推动它们
TG_PostPhysics    传感器采样、ROS 发布
```

每个求解器有自己的固定步长和一个 uBot 采样调度器：

- 求解器的时钟在它第一次有物体、且子系统第一次运行时启动，第一步在启动时间加一个步长处结束，之后都落在这条网格上。所以求解器的时间**从不超过**仿真时间，暂停期间不步进，恢复后也不补跑。
- 一帧需要的步数超过每帧步数上限（C++ 里 `GetMaxStepsPerFrame` / `SetMaxStepsPerFrame`，默认 1000，至少为 1）时，最早的那些步照常执行，其余的被跳过，计入 `Get Dropped Physics Steps`。跳过的步不补跑，求解器从此少走了这些步，见“单向作用和限制”。
- 一个求解器的物体都解绑后，它不再步进；下一次有物体绑定时时钟重新启动。丢弃的步数累计，不清零。
- **每帧写回一次，在所有求解器都步进完之后。** 对每个已有状态的被绑定物体，写回它的最新位姿：`SetWorldLocationAndRotation`（不扫掠，传送类型为 `None`，所以运动学体可以推动 Chaos 物体），并设置 `ComponentVelocity`（cm/s）。这一帧里它的求解器没有步进（帧比步长短）也照样写，所以别的东西挪动了组件，下一帧它也会回到求解器给出的位姿。非有限的状态不写回，并为这个求解器记一条 Warning。
- **写回的顺序：附着父级在前，子级在后。** 移动一个组件会带着附着在它下面的组件一起动，所以按附着深度（到根要经过几个附着父级）从小到大写，跨后端也一样；这样父子两个组件都被绑定时（即使属于不同的后端），子组件最后落在自己求解器给出的位姿上，而不是被父组件带偏。同一深度的组件互不影响，按求解器和句柄排序，每次运行顺序相同。
- **写回途中解绑、重新绑定或销毁，不会写入过时的数据。** 移动一个组件会运行 UE 的委托（重叠、变换回调等），用户的代码可能在其中解绑、重新绑定或销毁别的被绑定组件，也可能是这个组件自己。所以先收集每个物体的状态，并记下当时是谁持有它（绑定组件、后端、句柄、求解器对象和交出代数）；真正移动之前、设置 `ComponentVelocity` 之前各核对一次，持有者变了就跳过这个物体。交出代数是子系统在每次成功交出一个组件时从一个只增不减的计数器取的编号，所以解绑后又绑定，即使求解器把刚释放的句柄再发给了新物体（绑定组件、后端、句柄、求解器都和原来一样），也不会被当成原来的持有关系。已经解绑、已重新绑定（换了代数）或已被销毁的组件，不会再收到过时的位姿或速度。组件在自己的变换回调里解绑自己时，位姿已经写完，只有后面的速度被跳过。

### 单向作用和限制

- 后端的物体和 Chaos 的物体之间只有**单向**作用：后端物体是 Chaos 里的运动学体，能推动 Chaos 物体；Chaos 物体推不动后端物体。互相需要动态接触的物体必须交给同一个后端。
- 0.1.0 不把 UE 的场景几何交给后端：后端物体只和后端自己模型里的东西接触。能力声明里的 `bStaticSceneGeometry`、`bKinematicForeignBodies` 只是后端对自己的说明，本插件目前不使用它们。
- 控制不属于后端接口。后端自己的组件实现 UBotCore 的执行器接口（速度、姿态、电机等指令），把指令直接写进自己的求解器。
- 位姿回写以步为单位，不做外推。没有丢弃过步数时，组件的位姿落后仿真时间最多一个步长。丢弃发生之后（一帧需要的步数超过每帧步数上限，见“每帧时序”）这一点不再成立：被跳过的步不会补跑，此后求解器实际推进的步数比仿真时间经过的步数少了丢弃的那些，物体的运动比仿真时间慢，差距是丢弃步数乘以步长，不会自己消失，直到这个求解器的时钟重新启动（它的物体全部解绑后再绑定）。传给 `Step` 的结束时间和状态的时间戳仍落在原来的网格上，不晚于仿真时间。`Get Dropped Physics Steps` 大于 0 就说明发生过。

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

`FUBotPhysicsCapabilities` 的字段只会追加：`bRigidBodies`、`bArticulations`、`bMaximalCoordinateJoints`、`bContacts`、`bStaticSceneGeometry`、`bKinematicForeignBodies`、`bAccelerations`、`bDeterministic`、`bDifferentiable`、`bSoftBodies`、`bCloth`、`bParticles`，默认全是 `false`，后端只声明自己真的支持的。

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

## 给后端作者：C++ 接口（实验性）

接口在 `UBotPhysicsBackend.h`，在小版本之间可能变化。后端插件依赖 `UBotCore` 和 `UBotPhysics`，在自己的模块启动时注册工厂，关闭时清理：

```cpp
void FMyBackendModule::StartupModule()
{
    FUBotExtensionRegistry::Get().RegisterExtension(
        UBot::Physics::BackendExtensionPoint, TEXT("MyBackend"), MakeShared<FMyBackendFactory>());
}

void FMyBackendModule::ShutdownModule()
{
    FUBotExtensionRegistry::Get().UnregisterAllFromPackage(TEXT("MyBackend"));
}
```

需要实现两个接口：

```cpp
class FMyBackendFactory final : public IUBotPhysicsBackendFactory
{
public:
    virtual FName GetExtensionName() const override { return TEXT("MyBackend"); }   // 绑定组件里填的后端名
    virtual FUBotPhysicsCapabilities GetCapabilities() const override;
    virtual TSharedPtr<IUBotPhysicsSolver> CreateSolver(UWorld& World) override;     // 不能在这个世界运行时返回 nullptr，并自己写日志
};

class FMySolver final : public IUBotPhysicsSolver
{
public:
    virtual int32 AddBody(const FUBotPhysicsBodyDesc& Desc, FText& OutError) override;
    virtual void RemoveBody(int32 BodyHandle) override;
    virtual double GetStepSeconds() const override;
    virtual void Step(double StepEndTimeSeconds) override;
    virtual bool GetBodyState(int32 BodyHandle, FUBotBodyState& OutState) const override;
};
```

约定：

- **所有调用都在游戏线程。** 求解器由世界子系统持有，世界结束时释放。
- **回调里不要重入。** 工厂和求解器的回调（`CreateSolver`、`AddBody`、`RemoveBody`、`Step`、`GetBodyState`）不能同步地绑定或解绑物体（`Bind`、`Unbind`），也不能同步地注册或注销后端（`RegisterExtension`、`UnregisterExtension`、`UnregisterAllFromPackage`），直接调用和经由你调用的代码间接触发都一样。子系统对已知的几种情况做了防御：后端在 `CreateSolver` 或 `AddBody` 里注销或换掉自己（见下一条和“绑定与所有权”）、`CreateSolver` 里触发同一个后端的绑定（被拒绝）、写回途中用户代码解绑或重新绑定（不写过时的数据）。但这些只是兜底，这种重入不受支持，行为没有保证。需要这样做时，等回调返回之后再做，例如放到之后某一帧的 Tick 里。
- **注销。** 在后端模块的 `ShutdownModule` 里 `UnregisterAllFromPackage`（或 `UnregisterExtension`）注销工厂，不要让工厂比它的代码活得更久。注销的那次调用里（如果从游戏线程调用），子系统就会取回这个后端的所有物体，对每个物体调用求解器的 `GetBodyState`（取最后的速度交给 Chaos）和 `RemoveBody`，然后释放求解器；这时后端的代码必须还在。注册表从别的线程被修改时，子系统把这件事推迟到游戏线程，在那之前不碰求解器和组件，所以在这种情况下也要保证后端的代码活到游戏线程处理完。已注册的工厂被换成另一个同名的工厂对象，同样视为原后端被注销。注销也可能发生在你自己的 `CreateSolver` 或 `AddBody` 里（例如它们调用了会卸载后端的代码）：子系统在这两个调用返回后检查工厂和求解器是不是原来的，不是就用 `RemoveBody` 取回已经接受的物体，绑定失败为 `BackendMissing`，所以 `RemoveBody` 可能在工厂已被注销后，对一个刚刚还在 `AddBody` 里的求解器调用。这是对重入的兜底，不是约定：不要依赖它，见上一条。
- **`AddBody`。** 返回大于等于 0 的句柄，或 `INDEX_NONE` 并设置 `OutError`（会出现在绑定的错误里）。`FUBotPhysicsBodyDesc` 带组件、`Body Name`、发起绑定的绑定组件（可以转成后端自己的子类读模型设置）和组件在交出时的世界变换（UE 世界，厘米）。交给求解器时组件已经是运动学体。
- **`GetStepSeconds`。** 固定步长，求解器生命期内不变，范围 0.0001 s 到 100 s。
- **`Step`。** 前进一步；`StepEndTimeSeconds` 是这一步结束时的 uBot 仿真时间，从不晚于当前仿真时间。
- **`GetBodyState`。** 最近一步结束时的状态，UE 世界轴、SI 单位（位置 m，速度 m/s，角速度 rad/s，旋向与 UE 一致：+Z 向右偏航）；第一步之前或句柄未知时返回 `false`。子系统会把 `TimestampSeconds` 设为这一步结束的时间。
- **坐标。** 后端内部用右手系时，用 UBotCore 的 `UBot::RightHanded` 在 UE 轴和后端轴之间换算。
- **控制。** 不在这个接口里。后端自己的组件实现 UBotCore 的执行器接口，把指令直接送给自己的求解器。

`UBot::Physics::FindBackendFactory(Name)` 和 `UBot::Physics::GetBackendNames()` 查询已注册的后端。

## 常见问题

| 现象 | 原因和处理 |
| --- | --- |
| 绑定状态是 `BackendMissing` | `Backend` 拼写错误，没有填，或后端插件没有启用。错误里列出了已注册的后端名 |
| 绑定状态是 `Rejected`，错误说不是 Movable | 被绑定组件的 Mobility 必须是 `Movable` |
| 错误说已经绑定到某个后端 | 同一个组件只能在一个绑定里出现一次（也不能被两个绑定组件同时列出） |
| 错误说组件属于另一个 Actor | 绑定组件只能交出自己 Actor 的组件。要交出别的 Actor 的组件，给那个 Actor 加一个自己的绑定组件 |
| 绑定状态变成 `BackendMissing`，错误说后端已注销 | 后端插件在运行中卸载或被换掉。物体已经回到 Chaos；后端重新注册后再调用 `Bind` |
| 错误说已经被某某移动 | 这个组件已经由别的刚体状态源驱动（例如载具自己的积分器），不能再交给后端 |
| 错误说根组件不是 Primitive 组件 | `Bodies` 为空时用的是根组件，它必须是网格、盒体之类的 Primitive 组件；或者在 `Bodies` 里指定别的组件 |
| 错误里有后端自己的原因 | 求解器的 `AddBody` 拒绝了这个物体，例如它的模型里没有这个名字 |
| 错误说该后端的求解器还在创建 | 后端自己的 `CreateSolver` 里（直接或间接）调用了 `Bind`。回调里不能重入，见“给后端作者”；等 `CreateSolver` 返回之后再绑定 |
| 绑定成功但物体不动 | 后端的第一步在时钟启动后一个步长处结束；游戏暂停时也不会步进。再看后端自己的指令是否送达 |
| 日志提示后端的状态不是有限值 | 后端的求解发散了。这个物体的位姿不会写回，组件停在上一个位姿 |
| 解绑后物体飞走 | 解绑会把后端最后一步的速度交给 Chaos，这是预期的；要让物体停下，先让后端停下再解绑 |
| `Get Dropped Physics Steps` 在增长 | 一帧需要的步数超过每帧步数上限。帧太长，或后端的步长太短 |

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
- `Package.Descriptor`：`.uplugin` 的 `UBot` 块（层、对 UBotCore 的依赖、能力键、标签）。

测试在手动 tick 的临时 Game 世界里运行，用一个测试后端（每个物体匀速直线运动），不需要 GPU，也不需要真实的物理后端。需要在启用了 UBotCore 和 UBotPhysics 的工程里用带编辑器上下文的 `UnrealEditor` 运行。编辑器内：`Tools` 菜单打开 Session Frontend 的 Automation 标签页，过滤 `UBotPhysics`。命令行：

```powershell
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Project>\<Project>.uproject' -unattended -nop4 -nosplash -NullRHI '-ExecCmds=Automation RunTests UBotPhysics; Quit' '-TestExit=Automation Test Queue Empty'
```

把 `UBotPhysics` 换成更具体的前缀（如 `UBotPhysics.Binding`）可以只跑一类。UBotCore 自己的测试用 `UBotCore` 前缀。

## 版本与许可

当前版本 **0.1.0**（Beta，`IsBetaVersion`），仓库 <https://github.com/AstreoX/uBotPhysics>。

Copyright 2026 AstreoX。本项目以 [Apache License 2.0](LICENSE) 发布，可以自由使用、修改和再分发（包括商业用途），再分发时需保留 `LICENSE` 与 `NOTICE` 文件，并注明你修改过的文件。
