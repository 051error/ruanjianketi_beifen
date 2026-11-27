# 仓储多机器人 RCS 调度与动力学仿真系统 (WarehouseGridSimulation)

基于 **C# .NET 11**、**WinForms** 与 **SkiaSharp 纯矢量渲染** 开发的高性能仓储多机器人 (AGV/AMR) 控制系统 (RCS) 调度与高保真动力学仿真系统。

系统支持 **单机路径规划导航** 与 **大规模多机协同调度仿真**，具备插件化多智能体路径规划、三格移动闭塞交通管控、高精度物理动力学控制、多级死锁与长等待自主消解流水线，以及 60 FPS 亚像素平滑插值渲染。

---

## 一、系统架构分层设计

工程分为三大核心项目，严格遵循关注点分离与高内聚低耦合原则：

```
WarehouseGridSimulation.sln
│
├── ws/Core/          --> WarehouseGridSimulation.Core (纯仿真核心类库，net11.0)
│   ├── App/          --> 外部门面与线程安全交互外观 (SimulationApp)
│   ├── Collision/    --> 几何包络与扫掠连续碰撞检测 (CollisionDetector)
│   ├── Engine/       --> 引擎工作线程、权威快照发布与自愈流水线 (SimulationEngine)
│   ├── Geometry/     --> 2D 几何位姿、角度归一化与欧氏计算 (Pose2D, GridPoint)
│   ├── Map/          --> 正交网格地图、站点与拓扑管理 (GridMap, Station)
│   ├── Motion/       --> 底层闭环控制、PID 纠偏与指令下发 (MotionController, PidController)
│   ├── Planning/     --> 插件化多智能体时空路径规划核心 (CBS, ECBS, CA*, WHCA*)
│   ├── Robots/       --> 差速轮物理动力学模型与配置 (SimulatedRobot, RobotConfig)
│   ├── Tasks/        --> 仓储搬运作业生命周期管理 (TaskManager, WarehouseTask)
│   └── Traffic/      --> 三格移动闭塞、通道边锁与死锁拓扑诊断 (GridReservationManager, DeadlockManager)
│
├── ws/Skiasharp/     --> WarehouseGridSimulation.SkiaSharp (WinForms 宿主与矢量渲染呈现层)
│   ├── Controls/     --> SkiaCanvasControl 高性能自绘画布控件
│   ├── Forms/        --> 主界面窗体与参数交互面板 (MainForm)
│   ├── Rendering/    --> SkiaSharp 纯矢量高帧率光栅化渲染器 (SkiaWarehouseRenderer)
│   └── UI/           --> 视口控制、交互按钮与协同呈现器 (ViewportController, MultiRobotPresenter)
│
└── ws/Tests/         --> WarehouseGridSimulation.Tests (全量 MSTest 单元测试工程)
    └── WarehouseGridSimulationTests.cs (场景 1 ~ 57 全量严苛单元测试用例)
```

### 1. 核心仿真层 (`WarehouseGridSimulation.Core`)
* **零 UI 依赖**：纯 C# 算法与动力学实现，无任何 WinForms 或 SkiaSharp 依赖，支持 Headless 高速独立仿真；
* **后台自驱动**：内置高精度独立工作线程（10Hz / 100ms），自主推进物理时钟、移动闭塞与死锁诊断；
* **线程安全发布**：每次计算完成原子化向外部发布不可变无锁快照 `SimulationSnapshot`，附带高精度物理发布时间戳 `WallClockTimeSeconds`。

### 2. 渲染与呈现层 (`WarehouseGridSimulation.SkiaSharp`)
* **UI 零负担**：界面仅作为快照消费者，彻底卸载物理步进积分计算；
* **60 FPS 亚像素平滑插值 (LERP)**：利用快照时间戳计算插值系数 `interpolationAlpha`，小车在 60Hz 刷新率下实现平滑巡航过渡；
* **后台异步重规划**：路径寻路移入后台线程池异步执行，UI 渲染线程绝不阻塞卡顿。

### 3. 测试验证工程 (`WarehouseGridSimulation.Tests`)
* 覆盖从单车运动学、PID 归正、旋转对准、三格闭塞延迟释放、通道边锁对穿拦截、到外围单车破环、长等待自愈等 **59 个严苛业务场景**，全量测试 100% 绿色通过。

---

## 二、关键技术与核心机制解析

### 1. 交通资源管控：三格移动闭塞与精准定向防对穿锁
* **三格移动闭塞安全锁**：
  小车在正交网格间行驶时，采用**动力学最大制动距离 + 安全冗余步数（LookaheadAuthorizationSteps = 4）**进行前瞻授权。小车扫码确认进入目标格 $G_i$ 时立项物理占用，推迟一格释放原栅格和边资源。唯有进入下一新地标 $G_{i+1}$ 时，才释放身后 $G_{i-1}$，为小车尾部提供一整格身后绝对安全隔离区，彻底杜绝尾部追尾。
* **延迟释放模式 (ThreeGridDelayReleaseMode)**：
  - `ReleaseAllExceptDestinationOnArrival`（默认多机调度模式）：到达目标点停稳后释放除终点外的所有历史栅格与通道边锁；
  - `RetainPreviousGridOnDestination`（无尽巡航模式）：到达目标点后保留身后上一格占用以供下一航段闭塞衔接，但彻底清空所有历史通道边锁。
* **精准定向防对穿锁 (Directed Edge Lock)**：
  小车申请通道边 $(U \to V)$ 时，仅在检测到对方正持有相反方向逆穿占用 $(V \to U)$ 时坚决拦截（防正面对穿），而在目标格 $V$ 空出时允许同向跟进 $(U \to V)$，彻底解放同向跟车。
* **路径换载全量释放**：
  小车装载新航段前无条件彻底释放旧路径上持有的全部未来预约栅格与边锁，根绝僵尸锁（Ghost Locks）导致的死锁停滞。

### 2. 物理运动学与动力学闭环
* **梯形加减速与平滑刹停**：
  前进与旋转严格遵循动力学加减速与梯形减速曲线，进入制动区间后速度单调递减直至为 0，彻底杜绝末段反向加速。
* **转向与直行严格互斥**：
  自旋角速度未静止前拒绝接收前进位移；直行线速度未降至容差以内前拒绝接收旋转角度。起步直行时航向角偏差控制在 $< 0.01\text{ rad}$（约 0.57°），确保纯正交直线巡航无漂移。
* **转弯停靠 PID 纵向纠偏对准**：
  严格沿用闭环 PID 纵向纠偏控制（`_dockingPid`），拒绝瞬间瞬移或强制回中。在拐角节点转弯前，必须由 PID 驱动小车在栅格几何中心对齐停稳（偏差 $\le 8\text{mm}$）方可执行原地自旋，确保旋转扫掠外摆包络严格保留在自身网格安全边界内，从物理控制层杜绝偏心自旋剐蹭。

### 3. 多智能体时空路径规划 (MAPF) 核心
* **统一底层时空搜索核心 (`SpaceTimeAStar`)**：
  将无厚度的时间点离散化扩展为连续占用时间区间 $[t_{entry}, t_{exit} + buffer]$，支持转向惩罚代价（`TurnPenaltyCost`）与动态障碍物过滤。
* **4 大可插拔调度策略**：
  - `CbsPlanner`：基于冲突搜索的最优时空协同算法；
  - `EcbsPlanner`：引入次优启发式焦点搜索的增强型冲突搜索算法，毫秒级消解复杂机群冲突；
  - `CAStarPlanner`：连续预约时空 A* 算法；
  - `WhcaStarPlanner`：时间滑动窗口分层协同规划算法。
* **对角专属目标分配器 (`DiagonalTargetAllocator`)**：
  为机群各小车在其所属对角半区象限（对角互换原则）自适应寻找未被占用的安全专属网格。

### 4. 死锁诊断与多级自愈流水线
* **敏捷停滞感知**：
  死锁确认时间 0.4s，长等待超时阈值 3.5s，排除合法旋转与加速状态。
* **四级自愈应对策略**：
  1. **Tier 1 外围单车极速切环（耗时 < 1ms）**：基于 `EvaluateBestVictim` 挑选死锁环边缘小车侧向变道破环，内层机群瞬间全线自发疏通；
  2. **Tier 2 局部时空协同重规划**：局部死锁环时空协同重构（ECBS/WHCA*）；
  3. **Tier 3 全图全量异步重规划**：后台异步启动全量在途车协同重规划，物理仿真不停顿；
  4. **长等待与停泊车应急绕障 (`CascadingLongWaitBlockage`)**：非死锁但单向受阻超时（>3.5s）或被到站停泊车阻挡时，触发单车自主动态避障绕行。

### 5. 多机到站接力与 500ms 节流自愈重试池
* **单车到站即到即发**：无尽模式下单车到达目标点停稳瞬间触发 `RobotDestinationReached`，立刻沿用所选多机算法下发新航段；
* **500ms 重试时间节流 (Throttling)**：若因周围车流密集导致单车局部规划受阻，进入 `_pendingReassignRobots` 接力池，每 500ms 降频重试一次（降低 97% CPU 寻路峰值开销）；
* **后台异步规划 (`Task.Run`)**：路径搜索全在后台线程池执行，通过 `EnqueueCommand` 注入引擎，保证 60 FPS 满帧交互。

### 6. 双 HUD 实时监测仪表盘
* **集群宏观整体状态监测（未选中单车时）**：
  - 机器人数量（台）
  - 运行时间（秒）
  - 任务完成数（个）
  - 任务效率（每分钟完成任务数，按 60 秒整周期锁存更新）
  - 平均等待时间（秒）
  - 碰撞次数（次）
* **单机精细化状态监测（点击单台机器人后）**：
  - 机器人 ID（`#ID`）
  - 电池电量（`%`）
  - 扫描栅格坐标（`ConfirmedGrid`）
  - 当前线速度（`m/s`）
  - 航向朝向与方位（如 `90° (南)`）
  - 专属目标位置（`TargetGrid`）
  - 任务状态（`空闲`、`任务执行中`、**`等待资源 (#阻塞源ID)`**）
  - 载货状态（`空载` / `负载`）
* **一键切换**：支持车体与地标双重智能命中，点击空白网格随时切回集群整体状态。

---

## 三、快速上手与运行指南

### 1. 环境依赖
* **操作系统**：Windows 10 / 11 (x64)
* **运行环境**：[.NET 11 SDK](https://dotnet.microsoft.com/) (`11.0.100` 或更高版本)
* **GUI 框架**：Windows Forms (.NET 11-Windows)
* **渲染引擎**：SkiaSharp (纯 CPU/GPU 矢量光栅化)

### 2. 编译与测试

```powershell
# 1. 还原并编译整个解决方案 (0 警告 0 错误)
dotnet build ws/WarehouseGridSimulation.sln

# 2. 运行全量 59 项单元测试
dotnet test ws/WarehouseGridSimulation.sln --logger "console;verbosity=normal"
```

### 3. 运行主仿真程序

```powershell
dotnet run --project ws/Skiasharp/WarehouseGridSimulation.SkiaSharp.csproj
```

### 4. 界面交互与操作快捷键

| 操作 / 快捷键 | 功能说明 |
| :--- | :--- |
| **鼠标左键拖拽** | 视口平移（Pan） |
| **鼠标滚轮** | 视口平滑缩放（Zoom In / Out） |
| **鼠标单击小车** | 选中小车查看单机 8 大运行指标与阻塞源 ID，小车呈现高亮光晕 |
| **鼠标单击空白栅格** | 取消单车选中，切回机群整体状态监测 |
| **W / A / S / D 或 方向键** | 手动控制当前选中小车前进 / 左转 / 右转 |
| **S** | 单步物理步进推进（Step） |
| **R** | 重置小车位置与仿真状态（Reset） |
| **T** | 创建随机仓储搬运作业任务（Task） |
| **多机模式勾选“开启无尽模式”** | 开启单车到站即到即发持续对角巡航调度 |
