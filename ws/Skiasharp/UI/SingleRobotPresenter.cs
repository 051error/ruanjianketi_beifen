using System;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using WarehouseGridSimulation.Core.App;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Planning;

namespace WarehouseGridSimulation.Skia.UI
{
    /// <summary>
    /// 单机导航与交互逻辑呈现器 (SingleRobotPresenter)。
    /// 严格遵循架构职责划分：
    /// 1. 负责单机模式下目标网格点选、取消、移动中变道及状态提示的展示逻辑；
    /// 2. 所有调度操作均通过 SimulationApp 外观下沉至 Core 仿真引擎，UI 不直接操纵物理实体。
    /// </summary>
    public class SingleRobotPresenter
    {
        private readonly SimulationApp _app;
        private readonly Label _lblTargetInfo;
        private readonly Button _btnStartNavigation;
        private readonly ComboBox _cmbAlgorithm;

        public Point? SelectedTarget { get; private set; }

        public SingleRobotPresenter(
            SimulationApp app,
            Label lblTargetInfo,
            Button btnStartNavigation,
            ComboBox cmbAlgorithm)
        {
            // 初始化单机导航呈现器。
            // app: 仿真应用外观实例; lblTargetInfo: 目标状态提示标签; btnStartNavigation: 确定启动导航按钮; cmbAlgorithm: 寻路算法选择下拉框。
            _app = app ?? throw new ArgumentNullException(nameof(app));
            _lblTargetInfo = lblTargetInfo;
            _btnStartNavigation = btnStartNavigation;
            _cmbAlgorithm = cmbAlgorithm;
        }

        public IPathPlanner GetCurrentPlanner()
        {
            // 获取当前下拉框所选择的路径规划器实例。
            if (_cmbAlgorithm != null && _cmbAlgorithm.SelectedIndex == 1)
            {
                return new DijkstraPlanner();
            }
            return new AStarPlanner();
        }

        public void OnAlgorithmChanged()
        {
            // 响应算法下拉框切换并更新引擎默认规划器。
            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r != null)
            {
                var mc = _app.Engine.GetMotionController(r.Id);
                if (mc != null && mc.HasPath && !mc.IsPathCompleted)
                {
                    return; // 移动中锁定算法
                }
            }

            if (_app.Engine != null)
            {
                _app.Engine.PathPlanner = GetCurrentPlanner();
            }
        }

        public void OnGridClicked(Point clickedPoint)
        {
            // 处理用户在栅格地图中的点击事件。
            // clickedPoint: 用户鼠标点击选定的栅格坐标。
            if (SelectedTarget.HasValue && SelectedTarget.Value == clickedPoint)
            {
                HandleCancelNavigation();
                return;
            }

            SelectedTarget = clickedPoint;
            float cellSize = _app.CurrentMap?.CellSizeMeters ?? 0.55f;
            float physX = (clickedPoint.X + 0.5f) * cellSize;
            float physY = (clickedPoint.Y + 0.5f) * cellSize;

            if (_lblTargetInfo != null)
            {
                _lblTargetInfo.Text = $"目标网格: [{clickedPoint.X}, {clickedPoint.Y}] ({physX:F2}m, {physY:F2}m)";
                _lblTargetInfo.ForeColor = Color.FromArgb(0, 110, 220);
            }

            // 若机器人正在自动移动中，再次点击其它网格可即时换路
            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r != null)
            {
                var mc = _app.Engine.GetMotionController(r.Id);
                if (mc != null && mc.HasPath && !mc.IsPathCompleted)
                {
                    _app.NavigateRobot(r.Id, new GridPoint(clickedPoint.X, clickedPoint.Y), GetCurrentPlanner());
                }
            }
        }

        public void HandleStartNavigation(IWin32Window owner)
        {
            // 响应点击确定目标并自动移动操作。
            // owner: 弹窗宿主窗口对象。
            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r == null) return;

            var mc = _app.Engine.GetMotionController(r.Id);
            if (mc != null && mc.HasPath && !mc.IsPathCompleted)
            {
                return;
            }

            if (!SelectedTarget.HasValue)
            {
                MessageBox.Show(owner, "请先在地图中单击任意网格以设定目标点！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            Point target = SelectedTarget.Value;
            if (r.ConfirmedGrid.X == target.X && r.ConfirmedGrid.Y == target.Y)
            {
                MessageBox.Show(owner, "机器人当前已处于目标点网格！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            var result = _app.NavigateRobot(r.Id, new GridPoint(target.X, target.Y), GetCurrentPlanner());
            if (result.Success)
            {
                if (_lblTargetInfo != null)
                {
                    _lblTargetInfo.Text = $"导航中: [{r.ConfirmedGrid.X},{r.ConfirmedGrid.Y}] -> [{target.X},{target.Y}]";
                    _lblTargetInfo.ForeColor = Color.FromArgb(0, 140, 60);
                }
            }
            else
            {
                MessageBox.Show(owner, $"未找到到达目标点的连通路径！\n{result.Message}", "寻路失败", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }

        public void HandleCancelNavigation()
        {
            // 响应取消导航操作并通知机器人制动停在安全栅格。
            SelectedTarget = null;
            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r != null)
            {
                _app.CancelNavigation(r.Id);
            }

            if (_lblTargetInfo != null)
            {
                _lblTargetInfo.Text = "已取消导航 (停在安全栅格)";
                _lblTargetInfo.ForeColor = Color.FromArgb(200, 80, 40);
            }
        }

        public void ResetTarget()
        {
            // 复位目标选择与提示文本。
            SelectedTarget = null;
            if (_lblTargetInfo != null)
            {
                _lblTargetInfo.Text = "未设置目标 (在地图中单击网格)";
                _lblTargetInfo.ForeColor = Color.FromArgb(120, 130, 140);
            }
        }

        public void UpdateTickState()
        {
            // 随渲染定时器周期同步机器人的导航状态与控件交互可用性。
            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r == null) return;

            var mc = _app.Engine.GetMotionController(r.Id);
            bool isMoving = mc != null && mc.HasPath && !mc.IsPathCompleted;

            if (_btnStartNavigation != null && _btnStartNavigation.Enabled != !isMoving)
            {
                _btnStartNavigation.Enabled = !isMoving;
            }
            if (_cmbAlgorithm != null && _cmbAlgorithm.Enabled != !isMoving)
            {
                _cmbAlgorithm.Enabled = !isMoving;
            }

            if (isMoving && SelectedTarget.HasValue && _lblTargetInfo != null)
            {
                _lblTargetInfo.Text = $"自动导航中 -> [{SelectedTarget.Value.X}, {SelectedTarget.Value.Y}]";
                _lblTargetInfo.ForeColor = Color.FromArgb(0, 110, 220);
            }
            else if (!isMoving && SelectedTarget.HasValue &&
                     r.ConfirmedGrid.X == SelectedTarget.Value.X &&
                     r.ConfirmedGrid.Y == SelectedTarget.Value.Y)
            {
                if (_lblTargetInfo != null && !_lblTargetInfo.Text.StartsWith("已到达"))
                {
                    _lblTargetInfo.Text = $"已到达目标点 [{SelectedTarget.Value.X}, {SelectedTarget.Value.Y}]";
                    _lblTargetInfo.ForeColor = Color.FromArgb(40, 160, 80);
                }
            }
        }
    }
}
