using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;
using WarehouseGridSimulation.Core.App;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Robots;

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
        private readonly Stopwatch _navStopwatch = new Stopwatch();
        private readonly Random _random = new Random();

        public bool IsMultiNavigating { get; private set; }
        public IReadOnlyList<Point> MultiTargets => _multiTargets;

        public MultiRobotPresenter(SimulationApp app, Label lblMultiInfo)
        {
            // 初始化多机呈现器。
            // app: 仿真应用外观实例; lblMultiInfo: 界面多机状态提示标签。
            _app = app ?? throw new ArgumentNullException(nameof(app));
            _lblMultiInfo = lblMultiInfo;
        }

        public void ClearTargets()
        {
            // 清空多机专属目标集合并停止计时。
            _multiTargets.Clear();
            IsMultiNavigating = false;
            _navStopwatch.Reset();
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

            if (_lblMultiInfo != null)
            {
                _lblMultiInfo.Text = $"机群: {newRobots.Count}台 | 目标: {_multiTargets.Count}个 (就绪待命)";
                _lblMultiInfo.ForeColor = Color.FromArgb(40, 140, 60);
            }
        }

        public void GenerateDiagonalTargets()
        {
            // 为机群各机器人分配严格位于其对角半区的专属目标点。
            _multiTargets.Clear();
            var usedTargets = new HashSet<Point>();

            foreach (var robot in _app.Engine.Robots)
            {
                Point target = FindDiagonalTargetForRobot(robot, usedTargets);
                usedTargets.Add(target);
                _multiTargets.Add(target);
            }
        }

        private Point FindDiagonalTargetForRobot(SimulatedRobot robot, HashSet<Point> usedTargetPoints)
        {
            // 为单台机器人寻找对其所属对角半区未被占用的目标网格。
            // robot: 目标机器人实体; usedTargetPoints: 已被分配的目标点集合。
            int robotQuadX = (robot.ConfirmedGrid.X < 15) ? 0 : 1;
            int robotQuadY = (robot.ConfirmedGrid.Y < 15) ? 0 : 1;
            int diagQuadX = 1 - robotQuadX;
            int diagQuadY = 1 - robotQuadY;

            int minX = (diagQuadX == 0) ? 0 : 15;
            int maxX = (diagQuadX == 0) ? 14 : 29;
            int minY = (diagQuadY == 0) ? 0 : 15;
            int maxY = (diagQuadY == 0) ? 14 : 29;

            var candidates = new List<Point>();
            var fallbacks = new List<Point>();

            for (int x = minX; x <= maxX; x++)
            {
                for (int y = minY; y <= maxY; y++)
                {
                    Point pt = new Point(x, y);
                    if (usedTargetPoints.Contains(pt) || _app.CurrentMap.IsObstacle(new GridPoint(x, y)))
                        continue;

                    fallbacks.Add(pt);
                    if (Math.Abs(x - robot.ConfirmedGrid.X) > 10 || Math.Abs(y - robot.ConfirmedGrid.Y) > 10)
                    {
                        candidates.Add(pt);
                    }
                }
            }

            if (candidates.Count > 0)
            {
                return candidates[_random.Next(candidates.Count)];
            }
            if (fallbacks.Count > 0)
            {
                return fallbacks[_random.Next(fallbacks.Count)];
            }
            return new Point(robot.ConfirmedGrid.X, robot.ConfirmedGrid.Y);
        }

        public void HandleStartMultiNavigation(IWin32Window owner)
        {
            // 启动多机协同调度导航任务。
            // owner: 弹窗所属所有者窗口。
            if (_app.Engine.Robots.Count == 0 || _multiTargets.Count == 0)
            {
                MessageBox.Show(owner, "请先生成机群与目标点！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            for (int i = 0; i < _app.Engine.Robots.Count && i < _multiTargets.Count; i++)
            {
                var r = _app.Engine.Robots[i];
                var t = _multiTargets[i];
                var planResult = _app.Engine.PathPlanner.PlanPath(_app.CurrentMap, r.ConfirmedGrid, new GridPoint(t.X, t.Y));
                if (planResult.Success)
                {
                    var mc = _app.Engine.GetMotionController(r.Id);
                    mc?.SetPath(planResult.Path);
                }
            }

            IsMultiNavigating = true;
            _navStopwatch.Restart();
            _app.Engine.Start();

            if (_lblMultiInfo != null)
            {
                _lblMultiInfo.Text = $"多机协同导航执行中 ({_app.Engine.Robots.Count}台)...";
                _lblMultiInfo.ForeColor = Color.FromArgb(0, 110, 220);
            }
        }

        public void HandleStopMultiNavigation()
        {
            // 停止多机协同调度任务并暂停引擎推进。
            IsMultiNavigating = false;
            _navStopwatch.Stop();
            _app.Engine.Pause();

            if (_lblMultiInfo != null)
            {
                _lblMultiInfo.Text = $"多机调度已停止 (耗时 {_navStopwatch.Elapsed.TotalSeconds:F1}s)";
                _lblMultiInfo.ForeColor = Color.FromArgb(200, 80, 40);
            }
        }

        public void UpdateTickState()
        {
            // 随仿真时钟检查机群是否全部安全到达目标。
            if (!IsMultiNavigating || _app.Engine.Robots.Count == 0) return;

            bool allReached = true;
            for (int i = 0; i < _app.Engine.Robots.Count && i < _multiTargets.Count; i++)
            {
                var r = _app.Engine.Robots[i];
                var t = _multiTargets[i];
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
                if (_lblMultiInfo != null)
                {
                    _lblMultiInfo.Text = $"全部机群已到达专属目标点！(耗时 {_navStopwatch.Elapsed.TotalSeconds:F1}s)";
                    _lblMultiInfo.ForeColor = Color.FromArgb(40, 160, 80);
                }
            }
        }
    }
}
