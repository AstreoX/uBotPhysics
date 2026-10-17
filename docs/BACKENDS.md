# 给后端作者：C++ 接口（实验性）

下面的内容移自重写前的 README。使用手册见文档站 https://astreox.github.io/uBotDocs/physics/。

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
    virtual void UpdateScene(const FUBotPhysicsSceneUpdate& Update) override;   // 可选，默认什么也不做
};
```

约定：

- **所有调用都在游戏线程。** 求解器由世界子系统持有，世界结束时释放。
- **回调里不要重入。** 工厂和求解器的回调（`CreateSolver`、`AddBody`、`RemoveBody`、`Step`、`GetBodyState`、`UpdateScene`）不能同步地绑定或解绑物体（`Bind`、`Unbind`），也不能同步地注册或注销后端（`RegisterExtension`、`UnregisterExtension`、`UnregisterAllFromPackage`），直接调用和经由你调用的代码间接触发都一样。子系统对已知的几种情况做了防御：后端在 `CreateSolver` 或 `AddBody` 里注销或换掉自己（见下一条和“绑定与所有权”）、`CreateSolver` 里触发同一个后端的绑定（被拒绝）、写回途中用户代码解绑或重新绑定（不写过时的数据）。但这些只是兜底，这种重入不受支持，行为没有保证。需要这样做时，等回调返回之后再做，例如放到之后某一帧的 Tick 里。
- **注销。** 在后端模块的 `ShutdownModule` 里 `UnregisterAllFromPackage`（或 `UnregisterExtension`）注销工厂，不要让工厂比它的代码活得更久。注销的那次调用里（如果从游戏线程调用），子系统就会取回这个后端的所有物体，对每个物体调用求解器的 `GetBodyState`（取最后的速度交给 Chaos）和 `RemoveBody`，然后释放求解器；这时后端的代码必须还在。注册表从别的线程被修改时，子系统把这件事推迟到游戏线程，在那之前不碰求解器和组件，所以在这种情况下也要保证后端的代码活到游戏线程处理完。已注册的工厂被换成另一个同名的工厂对象，同样视为原后端被注销。注销也可能发生在你自己的 `CreateSolver` 或 `AddBody` 里（例如它们调用了会卸载后端的代码）：子系统在这两个调用返回后检查工厂和求解器是不是原来的，不是就用 `RemoveBody` 取回已经接受的物体，绑定失败为 `BackendMissing`，所以 `RemoveBody` 可能在工厂已被注销后，对一个刚刚还在 `AddBody` 里的求解器调用。这是对重入的兜底，不是约定：不要依赖它，见上一条。
- **`AddBody`。** 返回大于等于 0 的句柄，或 `INDEX_NONE` 并设置 `OutError`（会出现在绑定的错误里）。`FUBotPhysicsBodyDesc` 带组件、`Body Name`、发起绑定的绑定组件（可以转成后端自己的子类读模型设置）和组件在交出时的世界变换（UE 世界，厘米）。交给求解器时组件已经是运动学体。
- **`GetStepSeconds`。** 固定步长，求解器生命期内不变，范围 0.0001 s 到 100 s。
- **`Step`。** 前进一步；`StepEndTimeSeconds` 是这一步结束时的 uBot 仿真时间，从不晚于当前仿真时间。
- **`GetBodyState`。** 最近一步结束时的状态，UE 世界轴、SI 单位（位置 m，速度 m/s，角速度 rad/s，旋向与 UE 一致：+Z 向右偏航）；第一步之前或句柄未知时返回 `false`。子系统会把 `TimestampSeconds` 设为这一步结束的时间。
- **坐标。** 后端内部用右手系时，用 UBotCore 的 `UBot::RightHanded` 在 UE 轴和后端轴之间换算。
- **`UpdateScene`。** 只有工厂的能力声明了 `bStaticSceneGeometry`（收静态物体）或 `bKinematicForeignBodies`（收运动学物体）才会被调用，两项都没有就永远不会；求解器没有物体时也不调用。每帧最多一次，在这一帧的 `Step` 之前，有变化或者有运动学物体时才调用。`FUBotPhysicsSceneUpdate` 里：`SimTimeSeconds` 是这一帧的仿真时间；`Removed` 是离开这个求解器场景的物体 ID，先于 `Added` 处理；`Added` 是新进入场景的物体（`FUBotPhysicsSceneBody`：ID、组件的弱指针、静态还是运动学、物体坐标系在世界里的变换、形状、摩擦和弹性）；`KinematicPoses` 是更新之后这个求解器场景里**所有**运动学物体（刚加入的也在内）当前的位姿。ID 只指向一个组件的一次导出：组件的 Body Setup、世界缩放或 Mobility 变了，旧 ID 先随 `Removed` 离开，新的导出带新 ID 加入。后端的物体不会出现在自己的场景里，附着在它们下面的组件也不会。
- **形状和单位。** `FUBotPhysicsShape` 的单位是米，轴是 UE 世界轴，已经含组件的世界缩放，变换里没有缩放：`Sphere` 的 `Size.X` 是半径；`Box` 的 `Size` 是半边长；`Capsule` 的 `Size.X` 是半径，`Size.Z` 是两个球心距离的一半，轴是形状自己的 Z；`Convex` 的 `Vertices` 是凸包的点集；`TriangleMesh` 的 `Vertices` 和 `Indices`（每三个下标一个三角形）在物体坐标系里。`LocalTransform` 是形状相对物体坐标系的变换，凸包和三角网格是单位变换。后端给自己的物体取形状时，用同一个函数 `UBot::Physics::GetCollisionShapes(Component, OutShapes, OutSkippedShapes)`，导出的是 Chaos 模拟的那些形状；组件没有可导出的形状或类型不支持（蒙皮网格、实例化静态网格）时返回 `false`。
- **控制。** 不在这个接口里。后端自己的组件实现 UBotCore 的执行器接口，把指令直接送给自己的求解器。

`UBot::Physics::FindBackendFactory(Name)` 和 `UBot::Physics::GetBackendNames()` 查询已注册的后端。
