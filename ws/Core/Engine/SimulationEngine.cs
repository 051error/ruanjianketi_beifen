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
        public GridReservationManager ReservationManager { get; }
        public CollisionDetector CollisionDetector { get; }

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
            ReservationManager = new GridReservationManager();
            CollisionDetector = new CollisionDetector();

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

                var planResult = planner.PlanPath(Map, startGrid, target);
                if (!planResult.Success || planResult.Path == null || planResult.Path.Count == 0)
                {
                    return NavigationResult.Fail(string.IsNullOrEmpty(planResult.ErrorMessage) ? "未找到到达目标点的连通路径！" : planResult.ErrorMessage);
                }

                // 彻底释放机器人旧路径残留的所有预约栅格与未通行边，保留当前已占有的物理栅格
                ReservationManager.ReleaseRobotReservations(robotId, startGrid);

                int startTargetIdx = isOffCenter ? 0 : 1;
                controller.SetPath(planResult.Path, initialAuthorizedIndex: planResult.Path.Count - 1, startTargetIndex: startTargetIdx);

                // 单机模式下同步在管理器中锁定新路径全部预约资源
                if (_robots.Count <= 1)
                {
                    for (int i = 0; i < planResult.Path.Count - 1; i++)
                    {
                        var from = planResult.Path[i];
                        var to = planResult.Path[i + 1];
                        ReservationManager.TryAcquireStepAuthorization(robotId, from, to, out _);
                    }
                }

                if (!IsRunning || IsPaused)
                {
                    Start();
                }

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
                        _robots.Add(r);
                        _motionControllers[r.Id] = new MotionController(r.Id);
                        ReservationManager.RegisterRobotInitialGrid(r.Id, r.GridPosition);
                    }
                }
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
                    // 检查是否需要申请前方栅格与边的通行授权
                    // 单机无冲突场景全量预看授权以实现全速巡航，多机场景受预看步数防死锁约束
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

            // 步骤 7：独立碰撞检测 (与交通预约分离)
            var collisions = CollisionDetector.CheckCollisions(SimulationTime, _robots, Map);
            if (collisions.Count > 0)
            {
                foreach (var col in collisions)
                {
                    _recentCollisions.Add(col);
                    Log(col.ToString());
                    CollisionOccurred?.Invoke(col);
                }

                if (Config.AutoPauseOnCollision)
                {
                    foreach (var robot in _robots)
                    {
                        robot.EmergencyStop();
                    }
                    IsPaused = true;
                    Log("[安全保护] 检测到实体碰撞风险，仿真自动安全暂停");
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
                Robots = robotSnapshots,
                Tasks = taskSnapshots,
                OccupiedGrids = occupiedCopy,
                ReservedGrids = reservedCopy,
                CollisionEvents = collisionsCopy,
                LogEvents = logsCopy
            };
        }

        public void Log(string message)
        {
            string entry = $"[{SimulationTime:F1}s] {message}";
            _eventLogs.Add(entry);
            LogEmitted?.Invoke(entry);
        }
    }
}
