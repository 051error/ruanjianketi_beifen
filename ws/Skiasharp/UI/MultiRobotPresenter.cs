using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using WarehouseGridSimulation.Core.App;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Planning;
using WarehouseGridSimulation.Core.Robots;
using WarehouseGridSimulation.Core.Traffic;

namespace WarehouseGridSimulation.Skia.UI
{
    /// <summary>
    /// 多机调度协同交互呈现器 (MultiRobotPresenter)。
    /// 负责多机随机部署、对角目标分配、导航协同启停以及到达状态监控。
    /// </summary>
    public class MultiRobotPresenter
    {
        private readonly SimulationApp _app;
        private readonly Label _lblMultiInfo;
        private readonly List<Point> _multiTargets = new List<Point>();
        private readonly object _targetsLock = new object();
        private readonly Stopwatch _navStopwatch = new Stopwatch();
        private readonly Random _random = new Random();
        private readonly HashSet<int> _reassigningRobots = new HashSet<int>();
        private readonly HashSet<int> _pendingReassignRobots = new HashSet<int>();
        private IMultiAgentPathPlanner _activePlanner;

        private bool _isEndlessMode = false;
        private int _lastSampledMinute = 0;
        private int _lastCompletedCountAtMinute = 0;
        private double _lastRetryTimestampSeconds = 0.0;
        public int TotalGoalsReached { get; private set; } = 0;
        public double TaskEfficiencyPerMinute { get; private set; } = 0.0;

        public bool IsEndlessMode
        {
            get => _isEndlessMode;
            set
            {
                _isEndlessMode = value;
                if (_app.Engine != null)
                {
                    _app.Engine.Config.DelayReleaseMode = value
                        ? ThreeGridDelayReleaseMode.RetainPreviousGridOnDestination
                        : ThreeGridDelayReleaseMode.ReleaseAllExceptDestinationOnArrival;
                    _app.Engine.ReservationManager.DelayReleaseMode = _app.Engine.Config.DelayReleaseMode;
                }
            }
        }

        public bool IsMultiNavigating { get; private set; }
        public double NavElapsedTime => _navStopwatch.Elapsed.TotalSeconds;

        public int GetCompletedTasksCount()
        {
            // 获取多机调度模式下已完成的任务目标总数。
            if (IsEndlessMode)
            {
                return TotalGoalsReached;
            }

            if (_app.Engine == null || _app.Engine.Robots.Count == 0) return 0;

            var targets = MultiTargets;
            int count = 0;
            for (int i = 0; i < _app.Engine.Robots.Count && i < targets.Count; i++)
            {
                var r = _app.Engine.Robots[i];
                var t = targets[i];
                var mc = _app.Engine.GetMotionController(r.Id);
                if (r.ConfirmedGrid.X == t.X && r.ConfirmedGrid.Y == t.Y && (mc == null || mc.IsPathCompleted))
                {
                    count++;
                }
            }
            return count;
        }

        public IReadOnlyList<Point> MultiTargets
        {
            get
            {
                lock (_targetsLock)
                {
                    return _multiTargets.ToArray();
                }
            }
        }

        public MultiRobotPresenter(SimulationApp app, Label lblMultiInfo)
        {
            // 初始化多机呈现器。
            // app: 仿真应用外观实例; lblMultiInfo: 界面多机状态提示标签。
            _app = app ?? throw new ArgumentNullException(nameof(app));
            _lblMultiInfo = lblMultiInfo;
            if (_app.Engine != null)
            {
                _app.Engine.RobotDestinationReached += OnRobotDestinationReached;
            }
        }

        private void UpdateStatusLabelSafe(string text, Color color)
        {
            // 线程安全更新多机状态提示标签，杜绝后台工作线程直接跨线程修改 UI 控件引发崩溃。
            // text: 提示状态文本内容; color: 文本显示颜色。
            if (_lblMultiInfo == null) return;

            if (_lblMultiInfo.InvokeRequired)
            {
                try
                {
                    _lblMultiInfo.BeginInvoke(new Action(() =>
                    {
                        _lblMultiInfo.Text = text;
                        _lblMultiInfo.ForeColor = color;
                    }));
                }
                catch
                {
                    // 窗体关闭或销毁时静默忽略
                }
            }
            else
            {
                _lblMultiInfo.Text = text;
                _lblMultiInfo.ForeColor = color;
            }
        }

        public void ClearTargets()
        {
            // 清空多机专属目标集合并停止计时。
            lock (_targetsLock)
            {
                _multiTargets.Clear();
                _reassigningRobots.Clear();
                _pendingReassignRobots.Clear();
            }
            IsMultiNavigating = false;
            TotalGoalsReached = 0;
            _lastSampledMinute = 0;
            _lastCompletedCountAtMinute = 0;
            _lastRetryTimestampSeconds = 0.0;
            TaskEfficiencyPerMinute = 0.0;
            _navStopwatch.Reset();
            _app.Engine?.ResetCollisionStats();
        }

        public void GenerateRandomRobots(int count)
        {
            // 在地图中随机生成互不重叠的多机机群并分配对角专属目标点。
            // count: 待部署机器人总数量。
            if (_app.CurrentMap == null) return;

            ClearTargets();
            int totalCells = _app.CurrentMap.Width * _app.CurrentMap.Height;
            if (count > totalCells) count = totalCells;

            var usedPositions = new HashSet<int>();
            while (usedPositions.Count < count)
            {
                int cand = _random.Next(totalCells);
                int gx = cand % _app.CurrentMap.Width;
                int gy = cand / _app.CurrentMap.Height;
                if (!_app.CurrentMap.IsObstacle(new GridPoint(gx, gy)))
                {
                    usedPositions.Add(cand);
                }
            }

            var newRobots = new List<SimulatedRobot>();
            int rId = 1;
            foreach (int pos in usedPositions)
            {
                int gx = pos % _app.CurrentMap.Width;
                int gy = pos / _app.CurrentMap.Height;
                float heading = (float)(_random.Next(4) * Math.PI * 0.5f);
                var robot = new SimulatedRobot(rId++, new GridPoint(gx, gy), heading, _app.CurrentMap.CellSizeMeters);
                newRobots.Add(robot);
            }

            _app.Engine.SetRobots(newRobots);
            GenerateDiagonalTargets();

            UpdateStatusLabelSafe(
                $"机群: {newRobots.Count}台 | 目标: {MultiTargets.Count}个 (就绪待命)",
                Color.FromArgb(40, 140, 60));
        }

        public void GenerateDiagonalTargets()
        {
            // 为机群各机器人分配严格位于其对角半区的专属目标点。
            lock (_targetsLock)
            {
                _multiTargets.Clear();
                var usedTargets = new HashSet<Point>();

                foreach (var robot in _app.Engine.Robots)
                {
                    Point target = FindDiagonalTargetForRobot(robot, usedTargets);
                    usedTargets.Add(target);
                    _multiTargets.Add(target);
                }
            }
        }

        private Point FindDiagonalTargetForRobot(SimulatedRobot robot, HashSet<Point> usedTargetPoints)
        {
            // 为单台机器人寻找对其所属对角半区未被占用的目标网格。
            // robot: 目标机器人实体; usedTargetPoints: 已被分配的目标点集合。
            var usedGrids = new HashSet<GridPoint>();
            if (usedTargetPoints != null)
            {
                foreach (var pt in usedTargetPoints) usedGrids.Add(new GridPoint(pt.X, pt.Y));
            }
            var targetGrid = DiagonalTargetAllocator.FindDiagonalTarget(_app.CurrentMap, robot, usedGrids, _random);
            return new Point(targetGrid.X, targetGrid.Y);
        }

        public void HandleStartMultiNavigation(IWin32Window owner, IMultiAgentPathPlanner planner = null, MultiAgentPlanOptions options = null)
        {
            // 启动多机协同调度导航任务。
            // owner: 弹窗所属所有者窗口; planner: 选用的多智能体路径规划器; options: 算法配置参数。
            if (_app.Engine.Robots.Count == 0 || _multiTargets.Count == 0)
            {
                MessageBox.Show(owner, "请先生成机群与目标点！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            var chosenPlanner = planner ?? _app.Engine.MultiAgentPathPlanner ?? MultiAgentPlannerRegistry.Default;
            var opt = options ?? new MultiAgentPlanOptions();
            var tasks = new List<AgentTask>();
            for (int i = 0; i < _app.Engine.Robots.Count && i < _multiTargets.Count; i++)
            {
                var r = _app.Engine.Robots[i];
                var t = _multiTargets[i];
                tasks.Add(new AgentTask(r.Id, r.ConfirmedGrid, new GridPoint(t.X, t.Y)));
            }

            if (_lblMultiInfo != null)
            {
                _lblMultiInfo.Text = $"正在使用 {chosenPlanner.Name} 规划无冲突路径...";
                _lblMultiInfo.ForeColor = Color.FromArgb(0, 100, 200);
            }

            var planResult = chosenPlanner.Plan(_app.CurrentMap, tasks, opt);
            if (!planResult.Success || planResult.IsTimeout)
            {
                if (_lblMultiInfo != null)
                {
                    _lblMultiInfo.Text = $"任务失败: {planResult.Message}";
                    _lblMultiInfo.ForeColor = Color.FromArgb(220, 40, 40);
                }
                MessageBox.Show(
                    owner,
                    $"多机协同调度规划未成功！\n\n原因: {planResult.Message}\n求解耗时: {planResult.ElapsedMilliseconds} ms\n\n建议更换算法 (如 ECBS/WHCA*) 或重新随机生成机群分布。",
                    "调度失败提示",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Warning);
                return;
            }

            _activePlanner = chosenPlanner;

            // 规划成功，统一调用引擎标准居中对齐模型装载无冲突时空路径
            for (int i = 0; i < _app.Engine.Robots.Count && i < planResult.AgentPaths.Count; i++)
            {
                var r = _app.Engine.Robots[i];
                var path = planResult.AgentPaths[i];
                _app.Engine.LoadRobotPathWithCentering(r.Id, path, initialAuthorizedIndex: 0);
            }

            if (IsEndlessMode)
            {
                _app.Engine.Config.DelayReleaseMode = ThreeGridDelayReleaseMode.RetainPreviousGridOnDestination;
                _app.Engine.ReservationManager.DelayReleaseMode = ThreeGridDelayReleaseMode.RetainPreviousGridOnDestination;
            }

            IsMultiNavigating = true;
            _lastSampledMinute = 0;
            _lastCompletedCountAtMinute = 0;
            TaskEfficiencyPerMinute = 0.0;
            _navStopwatch.Restart();
            _app.Engine.Start();

            string modeText = IsEndlessMode ? "无尽模式持续协同巡航" : "协同行进";
            UpdateStatusLabelSafe(
                $"调度中: 采用 {chosenPlanner.Id} {modeText} ({planResult.ElapsedMilliseconds}ms)",
                Color.FromArgb(0, 110, 220));
        }

        public void HandleStopMultiNavigation()
        {
            // 停止多机协同调度任务并暂停引擎推进。
            IsMultiNavigating = false;
            _navStopwatch.Stop();
            _app.Engine.Pause();

            UpdateStatusLabelSafe(
                $"多机调度已停止 (耗时 {_navStopwatch.Elapsed.TotalSeconds:F1}s)",
                Color.FromArgb(200, 80, 40));
        }

        private void OnRobotDestinationReached(SimulatedRobot robot, GridPoint reachedGrid)
        {
            // 单台机器人到达其路径终点停稳时即时触发，无需等待全员到达，立刻派发新对角目标并沿用所选多机算法启程。
            // robot: 到达终点的机器人实体; reachedGrid: 机器人当前停靠的终点栅格。
            if (!IsMultiNavigating || !IsEndlessMode || _app.Engine == null || robot == null) return;

            lock (_targetsLock)
            {
                if (_reassigningRobots.Contains(robot.Id)) return;
                _reassigningRobots.Add(robot.Id);
            }

            int robotIdx = -1;
            for (int i = 0; i < _app.Engine.Robots.Count; i++)
            {
                if (_app.Engine.Robots[i].Id == robot.Id)
                {
                    robotIdx = i;
                    break;
                }
            }
            if (robotIdx < 0 || robotIdx >= MultiTargets.Count)
            {
                lock (_targetsLock)
                {
                    _reassigningRobots.Remove(robot.Id);
                }
                return;
            }

            AssignNextTargetForRobot(robotIdx, robot, reachedGrid);
        }

        private void AssignNextTargetForRobot(int robotIdx, SimulatedRobot robot, GridPoint currentGrid)
        {
            // 为到达当前目标点的单台机器人分配新对角目标，并在后台异步规划新航段以保证 UI 流畅。
            // robotIdx: 目标机器人在机群列表中的索引; currentGrid: 机器人当前所在的物理栅格。
            var usedTargets = new HashSet<GridPoint>();
            lock (_targetsLock)
            {
                foreach (var pt in _multiTargets)
                {
                    usedTargets.Add(new GridPoint(pt.X, pt.Y));
                }
            }
            usedTargets.Remove(currentGrid);

            GridPoint newGoal = DiagonalTargetAllocator.FindDiagonalTarget(_app.CurrentMap, robot, usedTargets, _random);
            var planner = _activePlanner ?? _app.Engine.MultiAgentPathPlanner ?? MultiAgentPlannerRegistry.Default;
            var singleTask = new List<AgentTask> { new AgentTask(robot.Id, robot.ConfirmedGrid, newGoal) };
            var dynamicObstacles = _app.Engine.GetDynamicObstaclesForReplanning(robot.Id);
            var map = _app.CurrentMap;

            System.Threading.Tasks.Task.Run(() =>
            {
                try
                {
                    var planResult = planner.Plan(map, singleTask, new MultiAgentPlanOptions
                    {
                        TimeoutMilliseconds = 1500,
                        WhcaWindowK = 18,
                        EcbsSuboptimality = 1.30,
                        IsDynamicBlocked = pt => dynamicObstacles.Contains(pt)
                    });

                    if (planResult.Success && planResult.AgentPaths != null && planResult.AgentPaths.Count > 0)
                    {
                        var newPath = planResult.AgentPaths[0];

                        lock (_targetsLock)
                        {
                            if (robotIdx < _multiTargets.Count)
                            {
                                _multiTargets[robotIdx] = new Point(newGoal.X, newGoal.Y);
                            }
                            _pendingReassignRobots.Remove(robot.Id);
                        }

                        // 统一调用引擎并发命令队列装载新航段
                        _app.Engine.EnqueueCommand(() =>
                        {
                            _app.Engine.LoadRobotPathWithCentering(robot.Id, newPath, initialAuthorizedIndex: 0);
                        });

                        TotalGoalsReached++;
                        UpdateStatusLabelSafe(
                            $"无尽模式持续巡航中: {_app.Engine.Robots.Count}台机群 | 累计达成目标: {TotalGoalsReached}个 ({_navStopwatch.Elapsed.TotalSeconds:F1}s)",
                            Color.FromArgb(0, 130, 230));
                    }
                    else
                    {
                        // 局部规划受阻，加入重试池在后续周期持续自愈重试
                        lock (_targetsLock)
                        {
                            _pendingReassignRobots.Add(robot.Id);
                        }
                    }
                }
                finally
                {
                    lock (_targetsLock)
                    {
                        _reassigningRobots.Remove(robot.Id);
                    }
                }
            });
        }

        private void RetryPendingReassignments()
        {
            // 轮询并重试之前因临时受阻而未成功分配新航段的到达小车。
            if (!IsMultiNavigating || !IsEndlessMode || _app.Engine == null) return;

            // 500ms 重试时间节流控制，将 60Hz 的高频轮询降频至 2Hz，降低 97% CPU 寻路开销
            double now = _navStopwatch.Elapsed.TotalSeconds;
            if (now - _lastRetryTimestampSeconds < 0.5) return;
            _lastRetryTimestampSeconds = now;

            List<int> toRetry;
            lock (_targetsLock)
            {
                if (_pendingReassignRobots.Count == 0) return;
                toRetry = _pendingReassignRobots.ToList();
            }

            foreach (int rId in toRetry)
            {
                lock (_targetsLock)
                {
                    if (_reassigningRobots.Contains(rId)) continue;
                    _reassigningRobots.Add(rId);
                }

                var r = _app.Engine.Robots.FirstOrDefault(x => x.Id == rId);
                if (r == null)
                {
                    lock (_targetsLock)
                    {
                        _pendingReassignRobots.Remove(rId);
                        _reassigningRobots.Remove(rId);
                    }
                    continue;
                }

                int robotIdx = -1;
                for (int i = 0; i < _app.Engine.Robots.Count; i++)
                {
                    if (_app.Engine.Robots[i].Id == rId) { robotIdx = i; break; }
                }

                if (robotIdx >= 0 && robotIdx < MultiTargets.Count)
                {
                    AssignNextTargetForRobot(robotIdx, r, r.ConfirmedGrid);
                }
                else
                {
                    lock (_targetsLock)
                    {
                        _reassigningRobots.Remove(rId);
                    }
                }
            }
        }

        public void UpdateTickState()
        {
            // 随仿真时钟检查机群是否全部安全到达目标。
            if (!IsMultiNavigating || _app.Engine.Robots.Count == 0) return;

            CheckAndUpdateTaskEfficiency();

            if (IsEndlessMode)
            {
                UpdateEndlessModeTickState();
                return;
            }

            var targets = MultiTargets;
            bool allReached = true;
            for (int i = 0; i < _app.Engine.Robots.Count && i < targets.Count; i++)
            {
                var r = _app.Engine.Robots[i];
                var t = targets[i];
                var mc = _app.Engine.GetMotionController(r.Id);
                bool atGoal = r.ConfirmedGrid.X == t.X && r.ConfirmedGrid.Y == t.Y && (mc == null || mc.IsPathCompleted);
                if (!atGoal)
                {
                    allReached = false;
                    break;
                }
            }

            if (allReached)
            {
                IsMultiNavigating = false;
                _navStopwatch.Stop();
                UpdateStatusLabelSafe(
                    $"全部机群已到达专属目标点！(耗时 {_navStopwatch.Elapsed.TotalSeconds:F1}s)",
                    Color.FromArgb(40, 160, 80));
            }
        }

        private void CheckAndUpdateTaskEfficiency()
        {
            // 按整分钟周期检测并锁存机群每分钟完成任务效率。
            int currentMinute = (int)(_navStopwatch.Elapsed.TotalSeconds / 60.0);
            if (currentMinute > _lastSampledMinute)
            {
                int currentCompleted = GetCompletedTasksCount();
                int minuteDelta = currentMinute - _lastSampledMinute;
                int countDelta = currentCompleted - _lastCompletedCountAtMinute;
                TaskEfficiencyPerMinute = (double)countDelta / minuteDelta;
                _lastSampledMinute = currentMinute;
                _lastCompletedCountAtMinute = currentCompleted;
            }
        }

        private void UpdateEndlessModeTickState()
        {
            // 随仿真时钟只读刷新无尽模式界面提示文本，杜绝 UI 线程直接执行寻路与修改引擎状态。
            RetryPendingReassignments();
            UpdateStatusLabelSafe(
                $"无尽模式持续巡航中: {_app.Engine.Robots.Count}台机群 | 累计达成目标: {TotalGoalsReached}个 ({_navStopwatch.Elapsed.TotalSeconds:F1}s)",
                Color.FromArgb(0, 130, 230));
        }
    }
}
