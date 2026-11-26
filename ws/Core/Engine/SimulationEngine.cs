using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using WarehouseGridSimulation.Core.Collision;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;
using WarehouseGridSimulation.Core.Motion;
using WarehouseGridSimulation.Core.Planning;
using WarehouseGridSimulation.Core.Robots;
using WarehouseGridSimulation.Core.Tasks;
using WarehouseGridSimulation.Core.Traffic;

namespace WarehouseGridSimulation.Core.Engine
{
    /// <summary>
    /// 仿真核心引擎 (SimulationEngine)。
    /// 统一驱动所有机器人与系统组件，严格按时序闭环推进。
    /// 核心不能依赖 SkiaSharp 或任何窗口控件。
    /// </summary>
    public class SimulationEngine
    {
        public SimulationConfig Config { get; }
        public GridMap Map { get; private set; }
        public TaskManager TaskManager { get; }
        public IPathPlanner PathPlanner { get; set; }
        public IMultiAgentPathPlanner MultiAgentPathPlanner { get; set; }
        public GridReservationManager ReservationManager { get; }
        public CollisionDetector CollisionDetector { get; }
        public DeadlockManager DeadlockManager { get; }
        public DeadlockDiagnosticReport LatestDeadlockReport { get; private set; }
        private float _lastDeadlockCheckTime = -1f;

        public IReadOnlyList<SimulatedRobot> Robots => _robots;
        private readonly List<SimulatedRobot> _robots = new List<SimulatedRobot>();
        private readonly Dictionary<int, MotionController> _motionControllers = new Dictionary<int, MotionController>();

        private readonly ConcurrentQueue<Action> _commandQueue = new ConcurrentQueue<Action>();
        private readonly List<string> _eventLogs = new List<string>();
        private readonly List<CollisionEvent> _recentCollisions = new List<CollisionEvent>();

        private readonly object _stateLock = new object();

        public long Tick { get; private set; } = 0;
        public float SimulationTime { get; private set; } = 0f;
        public bool IsRunning { get; private set; } = false;
        public bool IsPaused { get; private set; } = false;

        public event Action<SimulationSnapshot> SnapshotPublished;
        public event Action<string> LogEmitted;
        public event Action<CollisionEvent> CollisionOccurred;

        public SimulationSnapshot LatestSnapshot { get; private set; }

        public SimulationEngine(GridMap map, SimulationConfig config = null)
        {
            Config = config ?? new SimulationConfig();
            Map = map ?? throw new ArgumentNullException(nameof(map));
            TaskManager = new TaskManager();
            PathPlanner = new AStarPlanner();
            MultiAgentPathPlanner = new MultiAgentPathPlanner();
            ReservationManager = new GridReservationManager();
            CollisionDetector = new CollisionDetector();
            DeadlockManager = new DeadlockManager();

            ReservationManager.LongWaitDetected += OnLongWaitWarning;
        }

        private void OnLongWaitWarning(int robotId, GridPoint grid, float waitTime, string reason)
        {
            Log($"[警告] 机器人 #{robotId} 在 {grid} 等待通行授权已达 {waitTime:F1}s: {reason}");
        }

        public void EnqueueCommand(Action action)
        {
            if (action != null)
            {
                _commandQueue.Enqueue(action);
            }
        }

        public void AddRobot(SimulatedRobot robot)
        {
            // 向仿真引擎注册并初始化单个机器人实体与关联运动控制器。
            // robot: 待添加的仿真机器人实体。
            lock (_stateLock)
            {
                if (robot == null) throw new ArgumentNullException(nameof(robot));
                _robots.Add(robot);
                _motionControllers[robot.Id] = new MotionController(robot.Id);
                ReservationManager.RegisterRobotInitialGrid(robot.Id, robot.GridPosition);
            }
        }

        public MotionController GetMotionController(int robotId)
        {
            // 获取指定机器人关联的底层运动控制器实例。
            // robotId: 目标机器人全局唯一编号。
            _motionControllers.TryGetValue(robotId, out var mc);
            return mc;
        }

        public SimulatedRobot GetRobot(int robotId)
        {
            // 获取指定机器人实体对象。
            return _robots.FirstOrDefault(r => r.Id == robotId);
        }

        public NavigationResult NavigateRobot(int robotId, GridPoint target, IPathPlanner customPlanner = null)
        {
            // 为指定机器人规划并下发导航路径并自动启动仿真推进。
            // target: 导航目标网格坐标; customPlanner: 可选的定制路径规划器。
            lock (_stateLock)
            {
                if (Map == null)
                {
                    return NavigationResult.Fail("地图未初始化");
                }
                if (!Map.IsInside(target))
                {
                    return NavigationResult.Fail($"目标点 [{target.X}, {target.Y}] 超出地图范围");
                }
                if (Map.IsObstacle(target))
                {
                    return NavigationResult.Fail($"目标点 [{target.X}, {target.Y}] 位于障碍物内");
                }

                var robot = GetRobot(robotId);
                if (robot == null)
                {
                    return NavigationResult.Fail($"未找到编号为 #{robotId} 的机器人");
                }

                if (!_motionControllers.TryGetValue(robotId, out var controller))
                {
                    return NavigationResult.Fail($"未找到机器人 #{robotId} 的运动控制器");
                }

                float cellSize = Map.CellSizeMeters;
                float confirmedCenterX = (robot.ConfirmedGrid.X + 0.5f) * cellSize;
                float confirmedCenterY = (robot.ConfirmedGrid.Y + 0.5f) * cellSize;
                float offsetFromConfirmed = robot.Position.DistanceTo(confirmedCenterX, confirmedCenterY);

                bool isOffCenter = offsetFromConfirmed > robot.Config.PositionTolerance;
                GridPoint startGrid = robot.ConfirmedGrid;

                if (isOffCenter)
                {
                    // 机器人处于非中心偏位或田字格交汇处：启动方案一安全交叠加锁与就近归正
                    var overlappingGrids = GridReservationManager.GetOverlappingGrids(
                        robot.Position.X, robot.Position.Y, robot.Config.CollisionRadius, cellSize);

                    var validCandidates = overlappingGrids
                        .Where(g => Map.IsInside(g) && !Map.IsObstacle(g))
                        .ToList();

                    if (validCandidates.Count > 0)
                    {
                        if (!ReservationManager.TryAcquireMultiGridOverlapLock(robotId, validCandidates, out string overlapFail))
                        {
                            return NavigationResult.Fail($"机器人处于交汇区域，安全包络锁定失败: {overlapFail}");
                        }

                        // 启发式择优归正锚点：优先朝向目标终点、次选欧氏距离最近且未被其他车占用的候选网格
                        startGrid = validCandidates
                            .Where(g => ReservationManager.CanEnterGrid(robotId, g, out _))
                            .OrderBy(g => Math.Abs(g.X - target.X) + Math.Abs(g.Y - target.Y))
                            .ThenBy(g => robot.Position.DistanceTo((g.X + 0.5f) * cellSize, (g.Y + 0.5f) * cellSize))
                            .FirstOrDefault();

                        if (startGrid == GridPoint.Empty)
                        {
                            startGrid = validCandidates[0];
                        }
                    }
                }

                if (startGrid == target && (!isOffCenter && (controller.IsPathCompleted || !controller.HasPath)))
                {
                    return NavigationResult.Fail("机器人当前已处于目标点网格！");
                }

                var planner = customPlanner ?? PathPlanner;
                if (planner == null)
                {
                    return NavigationResult.Fail("未配置有效的路径规划算法");
                }

                var impassable = GetImpassableGrids(robotId);
                var planResult = planner.PlanPath(Map, startGrid, target, pt => pt != target && impassable.Contains(pt));
                if (!planResult.Success || planResult.Path == null || planResult.Path.Count == 0)
                {
                    return NavigationResult.Fail(string.IsNullOrEmpty(planResult.ErrorMessage) ? "未找到到达目标点的连通路径！" : planResult.ErrorMessage);
                }

                // 彻底释放机器人旧路径残留的所有预约栅格与未通行边，保留当前已占有的物理栅格
                ReservationManager.ReleaseRobotReservations(robotId, startGrid);

                int startTargetIdx = isOffCenter ? 0 : 1;

                // 单机独占模式下全量锁定目标路径，多机调度模式下由 3 格动力学移动闭塞窗口动态申请
                if (_robots.Count <= 1)
                {
                    controller.SetPath(planResult.Path, initialAuthorizedIndex: planResult.Path.Count - 1, startTargetIndex: startTargetIdx);
                    for (int i = 0; i < planResult.Path.Count - 1; i++)
                    {
                        var from = planResult.Path[i];
                        var to = planResult.Path[i + 1];
                        ReservationManager.TryAcquireStepAuthorization(robotId, from, to, out _);
                    }
                }
                else
                {
                    controller.SetPath(planResult.Path, initialAuthorizedIndex: 0, startTargetIndex: startTargetIdx);
                    int lookaheadSteps = Config.LookaheadAuthorizationSteps;
                    while (controller.AuthorizedIndex < controller.CurrentPath.Count - 1 &&
                           controller.AuthorizedIndex < controller.CurrentTargetIndex + lookaheadSteps)
                    {
                        int fromIndex = controller.AuthorizedIndex;
                        int toIndex = fromIndex + 1;
                        GridPoint fromGrid = controller.CurrentPath[fromIndex];
                        GridPoint toGrid = controller.CurrentPath[toIndex];

                        if (ReservationManager.TryAcquireStepAuthorization(robotId, fromGrid, toGrid, out _))
                        {
                            controller.UpdateAuthorization(toIndex);
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                if (!IsRunning || IsPaused)
                {
                    Start();
                }

                // 每次重设目标点将碰撞统计置零
                ResetCollisionStats();

                PublishSnapshot();
                return NavigationResult.Ok(planResult.Path, $"成功启动导航 -> [{target.X}, {target.Y}]");
            }
        }

        public bool CancelNavigation(int robotId)
        {
            // 取消指定机器人的当前导航路径并下发制动信号。
            lock (_stateLock)
            {
                var robot = GetRobot(robotId);
                if (robot == null) return false;

                if (_motionControllers.TryGetValue(robotId, out var controller))
                {
                    controller.ClearPath();
                }

                // 取消导航时释放该机器人前方所有尚未通行的预约资源
                ReservationManager.ReleaseRobotReservations(robotId, robot.ConfirmedGrid);

                robot.ExecuteCommand(RobotCommand.Brake());
                PublishSnapshot();
                return true;
            }
        }

        public MultiAgentPlanResult PlanMultiAgentPaths(IReadOnlyList<AgentTask> tasks, MultiAgentPlanOptions options = null)
        {
            // 为多机器人机群规划无时空冲突的协同路径。
            // tasks: 各智能体起终点任务列表; options: 可选的多机规划参数。
            lock (_stateLock)
            {
                if (Map == null) return MultiAgentPlanResult.Fail("地图未初始化");
                if (MultiAgentPathPlanner == null) return MultiAgentPlanResult.Fail("未配置多智能体路径规划器");
                return MultiAgentPathPlanner.Plan(Map, tasks, options);
            }
        }

        public void SetRobots(IEnumerable<SimulatedRobot> robots)
        {
            // 重新批量部署并配置仿真机群。
            // robots: 待注入的仿真机器人集合。
            lock (_stateLock)
            {
                _robots.Clear();
                _motionControllers.Clear();
                ReservationManager.Clear();
                if (robots != null)
                {
                    foreach (var r in robots)
                    {
                        r.HasCollided = false;
                        _robots.Add(r);
                        _motionControllers[r.Id] = new MotionController(r.Id);
                        ReservationManager.RegisterRobotInitialGrid(r.Id, r.GridPosition);
                    }
                }
                CollisionDetector.Reset();
                _recentCollisions.Clear();
                PublishSnapshot();
            }
        }

        public void Start()
        {
            // 启动仿真时钟并开始物理时序推进。
            lock (_stateLock)
            {
                IsRunning = true;
                IsPaused = false;
                Log("仿真引擎已启动");
                PublishSnapshot();
            }
        }

        public void Pause()
        {
            // 暂停物理仿真时序推进并保持当前状态不变。
            lock (_stateLock)
            {
                IsPaused = true;
                Log("仿真引擎已暂停");
                PublishSnapshot();
            }
        }

        public void Resume()
        {
            // 恢复物理仿真时序的连续运行。
            lock (_stateLock)
            {
                if (!IsRunning)
                {
                    IsRunning = true;
                }
                IsPaused = false;
                Log("仿真引擎已恢复运行");
                PublishSnapshot();
            }
        }

        public void Reset(GridMap newMap = null)
        {
            // 将整个仿真引擎状态、时间、日志与机群重置复位。
            // newMap: 可选的新地图模型。
            lock (_stateLock)
            {
                IsRunning = false;
                IsPaused = false;
                Tick = 0;
                SimulationTime = 0f;
                _eventLogs.Clear();
                _recentCollisions.Clear();
                ReservationManager.Clear();
                CollisionDetector.Reset();
                DeadlockManager.Reset();
                LatestDeadlockReport = null;
                _lastDeadlockCheckTime = -1f;
                TaskManager.Clear();
                _motionControllers.Clear();
                _robots.Clear();

                if (newMap != null)
                {
                    Map = newMap;
                }

                Log("仿真引擎已重置");
                PublishSnapshot();
            }
        }

        public void ProcessPendingCommands()
        {
            while (_commandQueue.TryDequeue(out var cmd))
            {
                cmd?.Invoke();
            }
        }

        public SimulationSnapshot StepSingleTick()
        {
            lock (_stateLock)
            {
                ProcessPendingCommands();
                return ExecuteTick(forceSingleStep: true);
            }
        }

        public SimulationSnapshot Step()
        {
            lock (_stateLock)
            {
                ProcessPendingCommands();

                if (!IsRunning || IsPaused)
                {
                    return LatestSnapshot ?? BuildSnapshot();
                }
                return ExecuteTick(forceSingleStep: false);
            }
        }

        private SimulationSnapshot ExecuteTick(bool forceSingleStep)
        {
            // 步骤 1：处理外部排队命令 (已在入口调用 ProcessPendingCommands 统一处理)

            float dt = Config.FixedDeltaTime;
            float cellSize = Map.CellSizeMeters;

            // 步骤 2：读取一致的当前状态 & 更新任务分配
            TaskManager.AssignPendingTasks(_robots, Map, PathPlanner, (robot, task, path) =>
            {
                var controller = _motionControllers[robot.Id];
                // 初始授权第一格 (当前所在格)
                controller.SetPath(path, initialAuthorizedIndex: 0);
                Log($"任务 #{task.Id} 分配给机器人 #{robot.Id}，前往上货点 {task.PickStation.Name}");
            });

            // 步骤 3：更新在途任务状态 (模拟上货/卸货计时)
            TaskManager.UpdateActiveTasks(dt, _robots, Map, PathPlanner,
                onProceedToDrop: (robot, task, path) =>
                {
                    var controller = _motionControllers[robot.Id];
                    controller.SetPath(path, initialAuthorizedIndex: 0);
                    Log($"机器人 #{robot.Id} 完成上货，装载货物前往卸货点 {task.DropStation.Name}");
                },
                onTaskCompleted: (robot, task) =>
                {
                    var controller = _motionControllers[robot.Id];
                    controller.ClearPath();
                    Log($"机器人 #{robot.Id} 完成卸货，任务 #{task.Id} 已顺利完成！");
                });

            // 步骤 4：处理交通预约授权
            foreach (var robot in _robots)
            {
                if (_motionControllers.TryGetValue(robot.Id, out var controller) && controller.HasPath && !controller.IsPathCompleted)
                {
                    // 单机场景全量预看以实现极速巡航，多机场景采用动力学最大制动距离+1格安全冗余 (3格移动闭塞)
                    int lookaheadSteps = (_robots.Count <= 1) ? 9999 : Config.LookaheadAuthorizationSteps;
                    while (controller.AuthorizedIndex < controller.CurrentPath.Count - 1 &&
                           controller.AuthorizedIndex < controller.CurrentTargetIndex + lookaheadSteps)
                    {
                        int fromIndex = controller.AuthorizedIndex;
                        int toIndex = fromIndex + 1;
                        GridPoint fromGrid = controller.CurrentPath[fromIndex];
                        GridPoint toGrid = controller.CurrentPath[toIndex];

                        if (ReservationManager.TryAcquireStepAuthorization(robot.Id, fromGrid, toGrid, out string failReason))
                        {
                            controller.UpdateAuthorization(toIndex);
                        }
                        else
                        {
                            ReservationManager.AccumulateWaitTime(robot.Id, fromGrid, dt, failReason);
                            break; // 前方受阻，等待释放
                        }
                    }
                }
            }

            // 步骤 4.1：周期性死锁拓扑诊断与在线协同重规划自主消解
            if (_robots.Count > 1 && (SimulationTime - _lastDeadlockCheckTime >= 0.2f || _lastDeadlockCheckTime < 0f))
            {
                _lastDeadlockCheckTime = SimulationTime;
                LatestDeadlockReport = DeadlockManager.Diagnose(_robots, ReservationManager, Map, SimulationTime, id => GetMotionController(id));
                if (Config.AutoReplanningOnDeadlock)
                {
                    if (LatestDeadlockReport.Status == DeadlockStatus.NeedGlobalReplanning)
                    {
                        TryResolveDeadlockByReplanning(LatestDeadlockReport);
                    }
                    else if (LatestDeadlockReport.Status == DeadlockStatus.StationaryBlockage)
                    {
                        TryResolveStationaryBlockageByReplanning(LatestDeadlockReport);
                    }
                }
            }

            // 步骤 5：生成机器人基础指令 (仅在拥有活动导航路径时由控制器下发指令，否则保留外部直接下发的手动控制信号)
            foreach (var robot in _robots)
            {
                if (_motionControllers.TryGetValue(robot.Id, out var controller) && controller.HasPath && !controller.IsPathCompleted)
                {
                    var cmd = controller.ComputeNextCommand(robot, cellSize, dt);
                    robot.ExecuteCommand(cmd);
                }
            }

            // 步骤 6：推进机器人运动动力学步进
            foreach (var robot in _robots)
            {
                robot.Step(dt, cellSize);
            }

            // 步骤 7：独立碰撞检测与局部急停隔离 (保持引擎调度简洁明了)
            var newCollisions = CollisionDetector.UpdateCollisions(SimulationTime, _robots, Map);
            foreach (var col in newCollisions)
            {
                _recentCollisions.Add(col);
                Log(col.ToString());
                CollisionOccurred?.Invoke(col);

                StopInvolvedRobot(col.RobotId1);
                if (col.RobotId2.HasValue)
                {
                    StopInvolvedRobot(col.RobotId2.Value);
                }
            }

            // 步骤 8：更新任务与资源占用 (基于地标确认执行双格安全锁交接与释放)
            foreach (var robot in _robots)
            {
                if (_motionControllers.TryGetValue(robot.Id, out var controller))
                {
                    if (controller.HasPath && controller.CurrentTargetIndex < controller.CurrentPath.Count)
                    {
                        GridPoint lastConfirmed = controller.LastConfirmedGrid;
                        GridPoint targetGrid = controller.CurrentPath[controller.CurrentTargetIndex];

                        ReservationManager.UpdateOccupancyAndRelease(
                            robot.Id,
                            lastConfirmed,
                            targetGrid,
                            robot.ConfirmedGrid);

                        ReservationManager.SweepHistoricalReservations(
                            robot.Id,
                            controller.CurrentPath,
                            robot.ConfirmedGrid);

                        if (robot.ConfirmedGrid == targetGrid)
                        {
                            controller.AdvanceConfirmedGrid(targetGrid);
                        }
                    }

                    // 检查机器人是否到达终点停稳
                    if (controller.IsPathCompleted && robot.MotionState == RobotMotionState.Stopped)
                    {
                        if (robot.CurrentTaskId.HasValue)
                        {
                            var task = TaskManager.Tasks.FirstOrDefault(t => t.Id == robot.CurrentTaskId.Value);
                            if (task != null)
                            {
                                if (task.Status == TaskStatus.MovingToPick && robot.GridPosition == task.PickStation.Position)
                                {
                                    task.Status = TaskStatus.WaitingPick;
                                    controller.ClearPath();
                                    Log($"机器人 #{robot.Id} 到达上货点 {task.PickStation.Name}，停稳等待上货");
                                }
                                else if (task.Status == TaskStatus.MovingToDrop && robot.GridPosition == task.DropStation.Position)
                                {
                                    task.Status = TaskStatus.WaitingDrop;
                                    controller.ClearPath();
                                    Log($"机器人 #{robot.Id} 到达卸货点 {task.DropStation.Name}，停稳等待卸货");
                                }
                            }
                        }
                    }
                }
            }

            // 步骤 9：更新仿真时间并发布权威快照
            SimulationTime += dt;
            Tick++;

            return PublishSnapshot();
        }

        public SimulationSnapshot PublishSnapshot()
        {
            var snapshot = BuildSnapshot();
            LatestSnapshot = snapshot;
            SnapshotPublished?.Invoke(snapshot);
            return snapshot;
        }

        public SimulationSnapshot BuildSnapshot()
        {
            var robotSnapshots = _robots.Select(r =>
            {
                _motionControllers.TryGetValue(r.Id, out var mc);
                return new RobotSnapshot
                {
                    Id = r.Id,
                    Position = r.Position,
                    PreviousPosition = r.PreviousPosition,
                    GridPosition = r.GridPosition,
                    LinearVelocity = r.LinearVelocity,
                    AngularVelocity = r.AngularVelocity,
                    MotionState = r.MotionState,
                    HasCargo = r.HasCargo,
                    CurrentTaskId = r.CurrentTaskId,
                    HasCollided = r.HasCollided,
                    CurrentPath = mc?.CurrentPath?.ToList() ?? new List<GridPoint>(),
                    CurrentTargetIndex = mc?.CurrentTargetIndex ?? 0,
                    TargetGrid = mc?.CurrentTargetGrid,
                    IsAwaitingAuthorization = mc?.IsAwaitingAuthorization ?? false,
                    WaitTimeSeconds = ReservationManager.GetRobotWaitTime(r.Id)
                };
            }).ToList();

            var taskSnapshots = TaskManager.Tasks.Select(t => new TaskSnapshot
            {
                Id = t.Id,
                PickName = t.PickStation.Name,
                PickPosition = t.PickStation.Position,
                DropName = t.DropStation.Name,
                DropPosition = t.DropStation.Position,
                Priority = t.Priority,
                Status = t.Status,
                AssignedRobotId = t.AssignedRobotId,
                WaitTimer = t.WaitTimer,
                FailureReason = t.FailureReason
            }).ToList();

            var occupiedCopy = new Dictionary<GridPoint, int>(ReservationManager.OccupiedGrids);
            var reservedCopy = new Dictionary<GridPoint, int>(ReservationManager.ReservedGrids);
            var collisionsCopy = _recentCollisions.TakeLast(20).ToList();
            var logsCopy = _eventLogs.TakeLast(30).ToList();

            return new SimulationSnapshot
            {
                Tick = Tick,
                SimulationTime = SimulationTime,
                IsRunning = IsRunning,
                IsPaused = IsPaused,
                TotalCollisionCount = CollisionDetector.TotalCollisionCount,
                IsDeadlockActive = LatestDeadlockReport?.IsDeadlockActive ?? false,
                DeadlockInfo = LatestDeadlockReport?.ResolutionDetails ?? string.Empty,
                Robots = robotSnapshots,
                Tasks = taskSnapshots,
                OccupiedGrids = occupiedCopy,
                ReservedGrids = reservedCopy,
                CollisionEvents = collisionsCopy,
                LogEvents = logsCopy
            };
        }

        private bool TryResolveDeadlockByReplanning(DeadlockDiagnosticReport report)
        {
            // 检测到死锁环时优先调度外围单车自主破环避让，失败时后备执行多智能体协同重规划。
            // report: 死锁诊断分析报告。
            if (report?.DeadlockCycles == null || report.DeadlockCycles.Count == 0 || MultiAgentPathPlanner == null)
            {
                return false;
            }

            // -------------------------------------------------------------
            // 第一级脱困策略：外围单车极速切环 (Victim Selection，耗时 < 1ms)
            // -------------------------------------------------------------
            foreach (var cycle in report.DeadlockCycles)
            {
                int victimId = cycle.BestVictimAgentId;
                if (victimId > 0 && PathPlanner != null)
                {
                    var victimRobot = GetRobot(victimId);
                    var victimMc = GetMotionController(victimId);
                    if (victimRobot != null && victimMc != null && victimMc.HasPath && victimMc.CurrentPath.Count > 0)
                    {
                        var target = victimMc.CurrentPath[^1];

                        var blockedByOthers = new HashSet<GridPoint>(GetImpassableGrids(victimId));
                        foreach (int otherId in cycle.Agents)
                        {
                            if (otherId != victimId)
                            {
                                var otherRobot = GetRobot(otherId);
                                if (otherRobot != null) blockedByOthers.Add(otherRobot.ConfirmedGrid);
                            }
                        }

                        var singleResult = PathPlanner.PlanPath(Map, victimRobot.ConfirmedGrid, target, pt => pt != target && blockedByOthers.Contains(pt));
                        if (singleResult.Success && singleResult.Path != null && singleResult.Path.Count > 0)
                        {
                            ReservationManager.ReleaseRobotReservations(victimId, victimRobot.ConfirmedGrid);
                            victimMc.SetPath(singleResult.Path, initialAuthorizedIndex: 0);

                            DeadlockManager.Reset();
                            _lastDeadlockCheckTime = SimulationTime;
                            Log($"[外围单车破环成功] 成功调度外围边缘小车 #{victimId} 侧向避让，死锁环瞬间打破，机群全线自发疏通！");
                            return true;
                        }
                    }
                }
            }

            // -------------------------------------------------------------
            // 第二级脱困策略：后备局部多智能体时空协同重规划 (ECBS/WHCA*)
            // -------------------------------------------------------------
            var involvedRobotIds = new HashSet<int>();
            foreach (var cycle in report.DeadlockCycles)
            {
                foreach (int rId in cycle.Agents)
                {
                    involvedRobotIds.Add(rId);
                }
            }

            var replanTasks = new List<AgentTask>();
            var replanRobots = new List<SimulatedRobot>();
            foreach (int rId in involvedRobotIds)
            {
                var r = GetRobot(rId);
                var mc = GetMotionController(rId);
                if (r != null && mc != null && mc.HasPath && mc.CurrentPath.Count > 0)
                {
                    var target = mc.CurrentPath[^1];
                    replanTasks.Add(new AgentTask(r.Id, r.ConfirmedGrid, target));
                    replanRobots.Add(r);
                }
            }

            if (replanTasks.Count < 2) return false;

            Log($"[死锁诊断] {report.ResolutionDetails}，正在启动在线协同重规划...");
            var blockedGrids = GetImpassableGrids();
            var replanResult = MultiAgentPathPlanner.Plan(Map, replanTasks, new MultiAgentPlanOptions
            {
                TimeoutMilliseconds = 1500,
                WhcaWindowK = 18,
                EcbsSuboptimality = 1.30,
                IsDynamicBlocked = pt => blockedGrids.Contains(pt)
            });

            if (replanResult.Success && replanResult.AgentPaths != null && replanResult.AgentPaths.Count == replanRobots.Count)
            {
                for (int i = 0; i < replanRobots.Count; i++)
                {
                    var r = replanRobots[i];
                    var newPath = replanResult.AgentPaths[i];
                    ReservationManager.ReleaseRobotReservations(r.Id, r.ConfirmedGrid);
                    var mc = GetMotionController(r.Id);
                    mc?.SetPath(newPath, initialAuthorizedIndex: 0);
                }

                DeadlockManager.Reset();
                _lastDeadlockCheckTime = SimulationTime;
                Log($"[死锁消解成功] 全局时空协同重规划成功 ({replanResult.ElapsedMilliseconds}ms)，机群重新解冻推进！");
                return true;
            }
            else
            {
                Log($"[死锁告警] 在线协同重规划无解，触发安全就地制动防死锁！");
                foreach (var r in replanRobots)
                {
                    r.ExecuteCommand(RobotCommand.Brake());
                }
                return false;
            }
        }

        public HashSet<GridPoint> GetImpassableGrids(int? excludeRobotId = null)
        {
            // 提取全图所有已碰撞急停、到站停靠或故障失能机器人的物理包络侵占网格集合。
            // excludeRobotId: 需排除的自身机器人编号。
            var blockedGrids = new HashSet<GridPoint>();
            float cellSize = Map?.CellSizeMeters ?? 0.55f;
            int width = Map?.Width ?? 30;
            int height = Map?.Height ?? 30;

            foreach (var r in _robots)
            {
                if (excludeRobotId.HasValue && r.Id == excludeRobotId.Value) continue;

                bool isStationary = r.LinearVelocity <= 0.05f;
                bool isStoppedOrFailed = r.MotionState == RobotMotionState.Stopped;
                bool isPathDone = _motionControllers.TryGetValue(r.Id, out var mc) && (mc.IsPathCompleted || mc.CurrentPath.Count == 0);

                if (isStationary && (isStoppedOrFailed || isPathDone))
                {
                    blockedGrids.Add(r.ConfirmedGrid);

                    // 物理包络全覆盖：根据机器人真实坐标与碰撞包络半径，提取车身侵占的所有相邻网格
                    float rRadius = r.Config?.CollisionRadius ?? 0.25f;
                    int minGx = Math.Max(0, (int)Math.Floor((r.Position.X - rRadius) / cellSize));
                    int maxGx = Math.Min(width - 1, (int)Math.Floor((r.Position.X + rRadius) / cellSize));
                    int minGy = Math.Max(0, (int)Math.Floor((r.Position.Y - rRadius) / cellSize));
                    int maxGy = Math.Min(height - 1, (int)Math.Floor((r.Position.Y + rRadius) / cellSize));

                    for (int gx = minGx; gx <= maxGx; gx++)
                    {
                        for (int gy = minGy; gy <= maxGy; gy++)
                        {
                            blockedGrids.Add(new GridPoint(gx, gy));
                        }
                    }
                }
            }
            return blockedGrids;
        }

        private bool TryResolveStationaryBlockageByReplanning(DeadlockDiagnosticReport report)
        {
            // 当在途机器人被已到站停靠的静止小车阻塞时自动为受阻车辆执行单车动态绕障重规划。
            // report: 死锁诊断分析报告。
            if (report?.StationaryBlockedAgents == null || report.StationaryBlockedAgents.Count == 0 || PathPlanner == null)
            {
                return false;
            }

            bool anyReplanned = false;
            var parkedGrids = GetImpassableGrids();

            foreach (int blockedId in report.StationaryBlockedAgents)
            {
                var r = GetRobot(blockedId);
                var mc = GetMotionController(blockedId);
                if (r == null || mc == null || !mc.HasPath || mc.CurrentPath.Count == 0) continue;

                var goal = mc.CurrentPath[^1];
                var planResult = PathPlanner.PlanPath(Map, r.ConfirmedGrid, goal, pt => pt != goal && parkedGrids.Contains(pt));
                if (planResult.Success && planResult.Path != null && planResult.Path.Count > 0)
                {
                    ReservationManager.ReleaseRobotReservations(r.Id, r.ConfirmedGrid);
                    mc.SetPath(planResult.Path, initialAuthorizedIndex: 0);
                    Log($"[自主绕障重规划] 机器人 #{r.Id} 成功重新规划避障路径绕开停泊车 -> [{goal.X}, {goal.Y}]！");
                    anyReplanned = true;
                }
                else
                {
                    Log($"[避障重规划受阻] 机器人 #{r.Id} 周围无可行绕行通道，就地等待安全释放！");
                }
            }

            if (anyReplanned)
            {
                DeadlockManager.Reset();
                _lastDeadlockCheckTime = SimulationTime;
            }
            return anyReplanned;
        }

        private void StopInvolvedRobot(int robotId)
        {
            // 对发生碰撞的涉事机器人执行局部安全急停并释放前方未通行预约。
            var robot = GetRobot(robotId);
            if (robot == null) return;

            robot.HasCollided = true;
            robot.EmergencyStop();
            if (_motionControllers.TryGetValue(robotId, out var controller))
            {
                controller.ClearPath();
            }
            ReservationManager.ReleaseRobotReservations(robotId, robot.ConfirmedGrid);
        }

        public void ResetCollisionStats()
        {
            // 重置碰撞检测器、清空碰撞历史记录并将所有小车的碰撞状态置零。
            lock (_stateLock)
            {
                CollisionDetector.Reset();
                _recentCollisions.Clear();
                foreach (var r in _robots)
                {
                    r.HasCollided = false;
                }
                PublishSnapshot();
            }
        }

        public void Log(string message)
        {
            string entry = $"[{SimulationTime:F1}s] {message}";
            _eventLogs.Add(entry);
            LogEmitted?.Invoke(entry);
        }
    }
}
