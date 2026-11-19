using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;
using SkiaSharp;
using WarehouseGridSimulation.Skia.Algorithms;
using WarehouseGridSimulation.Skia.Controls;
using WarehouseGridSimulation.Skia.Models;

namespace WarehouseGridSimulation.Skia.Forms
{
    /// <summary>
    /// 仓储物流多智能体调度仿真主窗体（基于 SkiaSharp 向量渲染）。
    /// </summary>
    public sealed class MainForm : Form
    {
        private const int GridColumns = 30;
        private const int GridRows = 30;
        private const float CellSizeMeters = 0.55f;
        private const float BaseCellPixels = 24f;
        private const float MinimumScale = 0.25f;
        private const float MaximumScale = 4f;

        private float _scale = 1f;
        private float _offsetX;
        private float _offsetY;
        private bool _isPanning;
        private Point _lastMousePosition;
        private Point _mousePressLocation;
        private bool _hasDraggedMouse;
        private bool _centerGridOnFirstResize = true;

        private Point? _selectedTarget;

        private readonly Robot _robot;
        private readonly Timer _simulationTimer;
        private readonly Stopwatch _stopwatch;
        private double _lastElapsedSeconds;

        private SkiaCanvasControl _skiaCanvas;
        private Panel _controlPanel;

        private ComboBox _cmbAlgorithm;
        private Label _lblTargetInfo;
        private Button _btnStartNavigation;
        private Button _btnCancelNavigation;
        private CheckBox _chkShowPath;

        private NumericUpDown _numSpeed;
        private TrackBar _trackSpeed;
        private NumericUpDown _numAccel;
        private TrackBar _trackAccel;
        private Button _btnMoveForward;
        private Button _btnTurnLeft;
        private Button _btnTurnRight;
        private Button _btnResetParams;
        private Button _btnResetPosition;

        public MainForm()
        {
            // 初始化主窗体、Skia 画布、机器人模型与控制面板
            Text = "仓储物流多智能体调度仿真 (SkiaSharp 寻路与自动导航)";
            Size = new Size(1180, 800);
            StartPosition = FormStartPosition.CenterScreen;
            MinimumSize = new Size(800, 600);
            DoubleBuffered = true;
            BackColor = Color.FromArgb(245, 247, 250);
            KeyPreview = true;

            _robot = new Robot(14, 14, CellSizeMeters, RobotOrientation.Up);
            _robot.NavStateChanged += OnRobotNavStateChanged;
            _robot.TargetReached += OnRobotTargetReached;

            InitializeComponents();

            _stopwatch = Stopwatch.StartNew();
            _lastElapsedSeconds = _stopwatch.Elapsed.TotalSeconds;

            _simulationTimer = new Timer();
            _simulationTimer.Interval = 16;
            _simulationTimer.Tick += OnSimulationTick;
            _simulationTimer.Start();
        }

        private void InitializeComponents()
        {
            // 创建并布局 Skia 画布与右侧控制交互面板。
            _controlPanel = new Panel();
            _controlPanel.Dock = DockStyle.Right;
            _controlPanel.Width = 290;
            _controlPanel.BackColor = Color.White;
            _controlPanel.BorderStyle = BorderStyle.FixedSingle;
            _controlPanel.AutoScroll = true;

            BuildControlPanelContent();

            _skiaCanvas = new SkiaCanvasControl();
            _skiaCanvas.Dock = DockStyle.Fill;
            _skiaCanvas.PaintSurface += OnPaintSkiaSurface;
            _skiaCanvas.MouseDown += OnCanvasMouseDown;
            _skiaCanvas.MouseMove += OnCanvasMouseMove;
            _skiaCanvas.MouseUp += OnCanvasMouseUp;
            _skiaCanvas.MouseWheel += OnCanvasMouseWheel;

            Controls.Add(_skiaCanvas);
            Controls.Add(_controlPanel);
        }

        private void BuildControlPanelContent()
        {
            // 构造右侧面板内的所有参数调节控件、动作按钮和状态卡片
            int currentY = 16;

            Label lblTitle = new Label();
            lblTitle.Text = "参数调节与寻路控制";
            lblTitle.Font = new Font("Microsoft YaHei", 10.5f, FontStyle.Bold);
            lblTitle.ForeColor = Color.FromArgb(30, 45, 60);
            lblTitle.Location = new Point(14, currentY);
            lblTitle.Size = new Size(255, 24);
            _controlPanel.Controls.Add(lblTitle);
            currentY += 34;

            _controlPanel.Controls.Add(CreateHorizontalSeparator(currentY));
            currentY += 14;

            // 1. 寻路算法选项
            Label lblAlgoTitle = new Label();
            lblAlgoTitle.Text = "寻路规划算法:";
            lblAlgoTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblAlgoTitle.Location = new Point(14, currentY);
            lblAlgoTitle.Size = new Size(250, 22);
            _controlPanel.Controls.Add(lblAlgoTitle);
            currentY += 24;

            _cmbAlgorithm = new ComboBox();
            _cmbAlgorithm.DropDownStyle = ComboBoxStyle.DropDownList;
            _cmbAlgorithm.Font = new Font("Microsoft YaHei", 9f);
            _cmbAlgorithm.Location = new Point(14, currentY);
            _cmbAlgorithm.Size = new Size(250, 26);
            _cmbAlgorithm.Items.Add("A* 算法 (曼哈顿距离启发式)");
            _cmbAlgorithm.Items.Add("Dijkstra 算法 (无启发式)");
            _cmbAlgorithm.SelectedIndex = 0;
            _cmbAlgorithm.SelectedIndexChanged += OnAlgorithmSelectedIndexChanged;
            _controlPanel.Controls.Add(_cmbAlgorithm);
            currentY += 36;

            // 2. 目标点设置与自动移动
            Label lblTargetGroupTitle = new Label();
            lblTargetGroupTitle.Text = "目标网格 (点击地图设置):";
            lblTargetGroupTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblTargetGroupTitle.Location = new Point(14, currentY);
            lblTargetGroupTitle.Size = new Size(250, 22);
            _controlPanel.Controls.Add(lblTargetGroupTitle);
            currentY += 24;

            _lblTargetInfo = new Label();
            _lblTargetInfo.Text = "未设置目标 (在地图中单击网格)";
            _lblTargetInfo.Font = new Font("Microsoft YaHei", 8.5f);
            _lblTargetInfo.ForeColor = Color.FromArgb(120, 130, 140);
            _lblTargetInfo.Location = new Point(14, currentY);
            _lblTargetInfo.Size = new Size(250, 24);
            _controlPanel.Controls.Add(_lblTargetInfo);
            currentY += 28;

            _btnStartNavigation = new Button();
            _btnStartNavigation.Text = "确定目标并自动移动";
            _btnStartNavigation.Location = new Point(14, currentY);
            _btnStartNavigation.Size = new Size(250, 32);
            _btnStartNavigation.BackColor = Color.FromArgb(230, 245, 255);
            _btnStartNavigation.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            _btnStartNavigation.ForeColor = Color.FromArgb(0, 100, 200);
            _btnStartNavigation.Click += delegate { HandleStartNavigation(); };
            _controlPanel.Controls.Add(_btnStartNavigation);
            currentY += 36;

            _btnCancelNavigation = new Button();
            _btnCancelNavigation.Text = "取消导航";
            _btnCancelNavigation.Location = new Point(14, currentY);
            _btnCancelNavigation.Size = new Size(250, 28);
            _btnCancelNavigation.Font = new Font("Microsoft YaHei", 8.5f);
            _btnCancelNavigation.Click += delegate { HandleCancelNavigation(); };
            _controlPanel.Controls.Add(_btnCancelNavigation);
            currentY += 34;

            _chkShowPath = new CheckBox();
            _chkShowPath.Text = "开启路径可视化轨迹";
            _chkShowPath.Font = new Font("Microsoft YaHei", 9f);
            _chkShowPath.Checked = true;
            _chkShowPath.Location = new Point(14, currentY);
            _chkShowPath.Size = new Size(250, 24);
            _chkShowPath.CheckedChanged += delegate { _skiaCanvas?.Invalidate(); };
            _controlPanel.Controls.Add(_chkShowPath);
            currentY += 32;

            _controlPanel.Controls.Add(CreateHorizontalSeparator(currentY));
            currentY += 14;

            // 3. 最大速度调节
            Label lblSpeedTitle = new Label();
            lblSpeedTitle.Text = "最大速度 (m/s):";
            lblSpeedTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblSpeedTitle.Location = new Point(14, currentY);
            lblSpeedTitle.Size = new Size(130, 24);
            _controlPanel.Controls.Add(lblSpeedTitle);

            _numSpeed = new NumericUpDown();
            _numSpeed.Location = new Point(155, currentY - 2);
            _numSpeed.Size = new Size(105, 24);
            _numSpeed.DecimalPlaces = 2;
            _numSpeed.Increment = 0.1m;
            _numSpeed.Minimum = 0.20m;
            _numSpeed.Maximum = 4.00m;
            _numSpeed.Value = 1.50m;
            _numSpeed.ValueChanged += OnNumSpeedValueChanged;
            _controlPanel.Controls.Add(_numSpeed);
            currentY += 32;

            _trackSpeed = new TrackBar();
            _trackSpeed.Location = new Point(10, currentY);
            _trackSpeed.Size = new Size(255, 45);
            _trackSpeed.Minimum = 20;
            _trackSpeed.Maximum = 400;
            _trackSpeed.TickFrequency = 40;
            _trackSpeed.Value = 150;
            _trackSpeed.Scroll += OnTrackSpeedScroll;
            _controlPanel.Controls.Add(_trackSpeed);
            currentY += 54;

            // 4. 加速度调节
            Label lblAccelTitle = new Label();
            lblAccelTitle.Text = "加速度 (m/s²):";
            lblAccelTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblAccelTitle.Location = new Point(14, currentY);
            lblAccelTitle.Size = new Size(130, 24);
            _controlPanel.Controls.Add(lblAccelTitle);

            _numAccel = new NumericUpDown();
            _numAccel.Location = new Point(155, currentY - 2);
            _numAccel.Size = new Size(105, 24);
            _numAccel.DecimalPlaces = 2;
            _numAccel.Increment = 0.2m;
            _numAccel.Minimum = 0.20m;
            _numAccel.Maximum = 6.00m;
            _numAccel.Value = 2.00m;
            _numAccel.ValueChanged += OnNumAccelValueChanged;
            _controlPanel.Controls.Add(_numAccel);
            currentY += 32;

            _trackAccel = new TrackBar();
            _trackAccel.Location = new Point(10, currentY);
            _trackAccel.Size = new Size(255, 45);
            _trackAccel.Minimum = 20;
            _trackAccel.Maximum = 600;
            _trackAccel.TickFrequency = 60;
            _trackAccel.Value = 200;
            _trackAccel.Scroll += OnTrackAccelScroll;
            _controlPanel.Controls.Add(_trackAccel);
            currentY += 54;

            _btnResetParams = new Button();
            _btnResetParams.Text = "恢复默认参数 (1.5m/s, 2.0m/s²)";
            _btnResetParams.Location = new Point(14, currentY);
            _btnResetParams.Size = new Size(250, 30);
            _btnResetParams.Font = new Font("Microsoft YaHei", 8.5f);
            _btnResetParams.Click += delegate
            {
                _numSpeed.Value = 1.50m;
                _numAccel.Value = 2.00m;
            };
            _controlPanel.Controls.Add(_btnResetParams);
            currentY += 42;

            _controlPanel.Controls.Add(CreateHorizontalSeparator(currentY));
            currentY += 14;

            // 5. 手动动作控制指令
            Label lblActions = new Label();
            lblActions.Text = "手动控制 (或使用键盘):";
            lblActions.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblActions.Location = new Point(14, currentY);
            lblActions.Size = new Size(250, 24);
            _controlPanel.Controls.Add(lblActions);
            currentY += 28;

            _btnMoveForward = new Button();
            _btnMoveForward.Text = "前进一格 [W / ↑]";
            _btnMoveForward.Location = new Point(14, currentY);
            _btnMoveForward.Size = new Size(250, 32);
            _btnMoveForward.BackColor = Color.FromArgb(235, 245, 255);
            _btnMoveForward.Font = new Font("Microsoft YaHei", 9f);
            _btnMoveForward.Click += delegate { HandleMoveForward(); };
            _controlPanel.Controls.Add(_btnMoveForward);
            currentY += 38;

            _btnTurnLeft = new Button();
            _btnTurnLeft.Text = "左转 90° [A / ←]";
            _btnTurnLeft.Location = new Point(14, currentY);
            _btnTurnLeft.Size = new Size(120, 32);
            _btnTurnLeft.Font = new Font("Microsoft YaHei", 9f);
            _btnTurnLeft.Click += delegate { HandleTurnLeft(); };
            _controlPanel.Controls.Add(_btnTurnLeft);

            _btnTurnRight = new Button();
            _btnTurnRight.Text = "右转 90° [D / →]";
            _btnTurnRight.Location = new Point(144, currentY);
            _btnTurnRight.Size = new Size(120, 32);
            _btnTurnRight.Font = new Font("Microsoft YaHei", 9f);
            _btnTurnRight.Click += delegate { HandleTurnRight(); };
            _controlPanel.Controls.Add(_btnTurnRight);
            currentY += 38;

            _btnResetPosition = new Button();
            _btnResetPosition.Text = "复位到中心 [空格]";
            _btnResetPosition.Location = new Point(14, currentY);
            _btnResetPosition.Size = new Size(250, 30);
            _btnResetPosition.Font = new Font("Microsoft YaHei", 8.5f);
            _btnResetPosition.Click += delegate { HandleResetPosition(); };
            _controlPanel.Controls.Add(_btnResetPosition);
        }

        private Panel CreateHorizontalSeparator(int top)
        {
            // 创建统一风格的水平分割线
            // top: 控件距离容器顶部的 Y 轴像素坐标
            Panel line = new Panel();
            line.Size = new Size(250, 1);
            line.Location = new Point(14, top);
            line.BackColor = Color.FromArgb(225, 230, 235);
            return line;
        }

        private void OnNumSpeedValueChanged(object sender, EventArgs e)
        {
            // 响应速度数字框修改并同步至机器人实体与滑动条
            // sender: 触发事件的对象源
            // e: 事件参数
            float newSpeed = (float)_numSpeed.Value;
            _robot.MaxSpeed = newSpeed;
            int trackVal = (int)Math.Round(newSpeed * 100f);
            if (trackVal < _trackSpeed.Minimum) trackVal = _trackSpeed.Minimum;
            if (trackVal > _trackSpeed.Maximum) trackVal = _trackSpeed.Maximum;
            if (_trackSpeed.Value != trackVal)
            {
                _trackSpeed.Value = trackVal;
            }
        }

        private void OnTrackSpeedScroll(object sender, EventArgs e)
        {
            // 响应速度滑动条滚动并同步至数字输入框
            decimal speedVal = (decimal)_trackSpeed.Value / 100m;
            if (_numSpeed.Value != speedVal)
            {
                _numSpeed.Value = speedVal;
            }
        }

        private void OnNumAccelValueChanged(object sender, EventArgs e)
        {
            // 响应加速度数字框修改并同步至机器人实体与滑动条
            float newAccel = (float)_numAccel.Value;
            _robot.Acceleration = newAccel;
            int trackVal = (int)Math.Round(newAccel * 100f);
            if (trackVal < _trackAccel.Minimum) trackVal = _trackAccel.Minimum;
            if (trackVal > _trackAccel.Maximum) trackVal = _trackAccel.Maximum;
            if (_trackAccel.Value != trackVal)
            {
                _trackAccel.Value = trackVal;
            }
        }

        private void OnTrackAccelScroll(object sender, EventArgs e)
        {
            // 响应加速度滑动条滚动并同步至数字输入框
            decimal accelVal = (decimal)_trackAccel.Value / 100m;
            if (_numAccel.Value != accelVal)
            {
                _numAccel.Value = accelVal;
            }
        }

        private void HandleMoveForward()
        {
            // 执行向前移动一格操作
            _robot.TryMoveForward(GridColumns, GridRows, CellSizeMeters);
        }

        private void HandleTurnLeft()
        {
            // 执行向左逆时针旋转 90 度操作
            _robot.TryTurnLeft();
        }

        private void HandleTurnRight()
        {
            // 执行向右顺时针旋转 90 度操作
            _robot.TryTurnRight();
        }

        private void HandleResetPosition()
        {
            // 将机器人复位回到网格中心点 (14, 14) 并清空目标标记。
            _selectedTarget = null;
            _lblTargetInfo.Text = "未设置目标 (在地图中单击网格)";
            _lblTargetInfo.ForeColor = Color.FromArgb(120, 130, 140);
            _robot.ResetToGrid(14, 14, CellSizeMeters);
            _skiaCanvas?.Invalidate();
        }

        private void SetSelectedTarget(Point target)
        {
            // 设置当前选定的目标网格并在移动状态下即时更换导航目标。
            // target: 用户在地图中选定的目标网格坐标。
            _selectedTarget = target;
            float physX = (target.X + 0.5f) * CellSizeMeters;
            float physY = (target.Y + 0.5f) * CellSizeMeters;
            _lblTargetInfo.Text = "目标网格: [" + target.X + ", " + target.Y + "] (" + physX.ToString("0.00") + "m, " + physY.ToString("0.00") + "m)";
            _lblTargetInfo.ForeColor = Color.FromArgb(0, 110, 220);

            if (_robot.NavState == RobotNavState.AutoMoving || _robot.NavState == RobotNavState.Braking)
            {
                _robot.SetNavigationTarget(target, GridColumns, GridRows, CellSizeMeters);
            }

            _skiaCanvas?.Invalidate();
        }

        private void HandleStartNavigation()
        {
            // 依据当前选定算法与目标点启动寻路移动任务。
            if (_robot.NavState != RobotNavState.Idle)
            {
                return;
            }

            if (!_selectedTarget.HasValue)
            {
                MessageBox.Show(this, "请先在地图中单击任意网格以设定目标点！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            Point target = _selectedTarget.Value;

            if (_robot.GridX == target.X && _robot.GridY == target.Y && _robot.NavState == RobotNavState.Idle)
            {
                MessageBox.Show(this, "机器人当前已处于目标点网格！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            if (_robot.NavState == RobotNavState.Idle)
            {
                PathAlgorithm algorithm = (_cmbAlgorithm.SelectedIndex == 1)
                    ? PathAlgorithm.Dijkstra
                    : PathAlgorithm.AStar;
                _robot.TrySetAlgorithm(algorithm);
            }

            bool success = _robot.SetNavigationTarget(target, GridColumns, GridRows, CellSizeMeters);
            if (!success)
            {
                MessageBox.Show(this, "未找到到达目标点的连通路径！", "寻路失败", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }

            _skiaCanvas?.Invalidate();
        }

        private void HandleCancelNavigation()
        {
            // 响应停止操作，清空目标标记并在下一个安全栅格平滑减速停下。
            _selectedTarget = null;
            _robot.StopAtNextGrid(CellSizeMeters);
            _lblTargetInfo.Text = "已取消目标点 (停在安全栅格)";
            _lblTargetInfo.ForeColor = Color.FromArgb(200, 80, 40);
            _skiaCanvas?.Invalidate();
        }

        private void OnRobotNavStateChanged(RobotNavState state)
        {
            // 响应机器人高层导航状态机状态变化并严格控制算法选择、确定目标按钮及动力学数值控件的使能。
            // state: 机器人最新的导航状态机状态。
            bool isIdle = (state == RobotNavState.Idle);

            if (_btnStartNavigation != null) _btnStartNavigation.Enabled = isIdle;
            if (_cmbAlgorithm != null) _cmbAlgorithm.Enabled = isIdle;
            if (_numSpeed != null) _numSpeed.Enabled = isIdle;
            if (_trackSpeed != null) _trackSpeed.Enabled = isIdle;
            if (_numAccel != null) _numAccel.Enabled = isIdle;
            if (_trackAccel != null) _trackAccel.Enabled = isIdle;
            if (_btnResetParams != null) _btnResetParams.Enabled = isIdle;

            // 同步右侧 GUI 交互界面的状态提示信息
            if (_lblTargetInfo != null)
            {
                switch (state)
                {
                    case RobotNavState.ManualMoving:
                        _lblTargetInfo.Text = "手动控制移动中 (键盘/按钮)";
                        _lblTargetInfo.ForeColor = Color.FromArgb(0, 120, 215);
                        break;
                    case RobotNavState.AutoMoving:
                        if (_selectedTarget.HasValue)
                        {
                            _lblTargetInfo.Text = "自动导航中 -> [" + _selectedTarget.Value.X + ", " + _selectedTarget.Value.Y + "]";
                            _lblTargetInfo.ForeColor = Color.FromArgb(0, 110, 220);
                        }
                        break;
                    case RobotNavState.Braking:
                        _lblTargetInfo.Text = "正在平滑减速刹停中...";
                        _lblTargetInfo.ForeColor = Color.FromArgb(220, 90, 40);
                        break;
                    case RobotNavState.Idle:
                        if (!_selectedTarget.HasValue)
                        {
                            _lblTargetInfo.Text = "未设置目标 (在地图中单击网格)";
                            _lblTargetInfo.ForeColor = Color.FromArgb(120, 130, 140);
                        }
                        break;
                }
            }

            _skiaCanvas?.Invalidate();
        }

        private void OnAlgorithmSelectedIndexChanged(object sender, EventArgs e)
        {
            // 响应寻路算法选择变更并在空闲状态下同步至机器人状态机。
            PathAlgorithm selectedAlgo = (_cmbAlgorithm.SelectedIndex == 1)
                ? PathAlgorithm.Dijkstra
                : PathAlgorithm.AStar;

            if (!_robot.TrySetAlgorithm(selectedAlgo))
            {
                MessageBox.Show(this, "机器人移动状态下禁止更改寻路算法，必须到达目标点后方可修改！", "状态机约束", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                _cmbAlgorithm.SelectedIndex = (_robot.CurrentAlgorithm == PathAlgorithm.Dijkstra) ? 1 : 0;
            }
        }

        private void OnRobotTargetReached()
        {
            // 响应机器人顺利抵达目标点事件并刷新状态提示。
            _lblTargetInfo.Text = "已到达目标点 [" + _robot.GridX + ", " + _robot.GridY + "]";
            _lblTargetInfo.ForeColor = Color.FromArgb(40, 160, 80);
            _skiaCanvas?.Invalidate();
        }

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            // 在窗体层全局拦截键盘输入以驱动机器人前进、转向及复位
            // msg: Windows 窗口消息引用
            // keyData: 按键代码及修饰键
            switch (keyData)
            {
                case Keys.W:
                case Keys.Up:
                    HandleMoveForward();
                    return true;

                case Keys.A:
                case Keys.Left:
                    HandleTurnLeft();
                    return true;

                case Keys.D:
                case Keys.Right:
                    HandleTurnRight();
                    return true;

                case Keys.Space:
                    HandleResetPosition();
                    return true;
            }

            return base.ProcessCmdKey(ref msg, keyData);
        }

        private void OnSimulationTick(object sender, EventArgs e)
        {
            // 驱动仿真时间步长并触发 Skia 画布重绘。
            double currentSeconds = _stopwatch.Elapsed.TotalSeconds;
            float deltaTime = (float)(currentSeconds - _lastElapsedSeconds);
            _lastElapsedSeconds = currentSeconds;

            if (deltaTime > 0.1f)
            {
                deltaTime = 0.1f;
            }

            _robot.Update(deltaTime, CellSizeMeters);
            _skiaCanvas.Invalidate();
        }

        protected override void OnResize(EventArgs e)
        {
            // 响应窗口尺寸调整并在首次加载时居中网格地图。
            base.OnResize(e);

            if (_centerGridOnFirstResize && _skiaCanvas != null && _skiaCanvas.Width > 0 && _skiaCanvas.Height > 0)
            {
                CenterGrid(_skiaCanvas.Width, _skiaCanvas.Height);
                _centerGridOnFirstResize = false;
            }
        }

        private void CenterGrid(int viewWidth, int viewHeight)
        {
            // 根据给定的视口长宽计算并居中网格位置
            // viewWidth: 画布可用宽度
            // viewHeight: 画布可用高度
            float gridSize = BaseCellPixels * GridColumns * _scale;
            _offsetX = (viewWidth - gridSize) * 0.5f;
            _offsetY = (viewHeight - gridSize) * 0.5f;
            if (_skiaCanvas != null)
            {
                _skiaCanvas.Invalidate();
            }
        }

        private void OnPaintSkiaSurface(SKCanvas canvas, int width, int height)
        {
            // 使用 SkiaSharp 绘制 30x30 离散栅格、规划路径、目标标记与机器人
            // canvas: Skia 目标渲染画布
            // width: 视口宽度
            // height: 视口高度
            canvas.Clear(new SKColor(245, 247, 250));

            float cellPixels = BaseCellPixels * _scale;
            float gridWidth = GridColumns * cellPixels;
            float gridHeight = GridRows * cellPixels;

            // 1. 绘制网格白色背景
            using (var floorPaint = new SKPaint
            {
                Color = SKColors.White,
                Style = SKPaintStyle.Fill
            })
            {
                canvas.DrawRect(_offsetX, _offsetY, gridWidth, gridHeight, floorPaint);
            }

            // 2. 绘制 30x30 栅格细线与深灰外框
            using (var gridPaint = new SKPaint
            {
                Color = new SKColor(220, 225, 230),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1f,
                IsAntialias = false
            })
            using (var borderPaint = new SKPaint
            {
                Color = new SKColor(50, 62, 72),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 2f,
                IsAntialias = true
            })
            {
                for (int col = 0; col <= GridColumns; col++)
                {
                    float x = _offsetX + col * cellPixels;
                    canvas.DrawLine(x, _offsetY, x, _offsetY + gridHeight, gridPaint);
                }

                for (int row = 0; row <= GridRows; row++)
                {
                    float y = _offsetY + row * cellPixels;
                    canvas.DrawLine(_offsetX, y, _offsetX + gridWidth, y, gridPaint);
                }

                canvas.DrawRect(_offsetX, _offsetY, gridWidth, gridHeight, borderPaint);
            }

            // 3. 高亮显示机器人当前所处网格
            using (var highlightPaint = new SKPaint
            {
                Color = new SKColor(0, 120, 215, 35),
                Style = SKPaintStyle.Fill
            })
            {
                float cellX = _offsetX + _robot.GridX * cellPixels;
                float cellY = _offsetY + _robot.GridY * cellPixels;
                canvas.DrawRect(cellX, cellY, cellPixels, cellPixels, highlightPaint);
            }

            // 4. 绘制规划路径轨迹折线（根据路径可视化选项决定是否显示）
            if (_chkShowPath != null && _chkShowPath.Checked)
            {
                DrawPlannedPath(canvas, cellPixels);
            }

            // 5. 绘制用户选定的目标点标记
            DrawTargetMarker(canvas, cellPixels);

            // 6. 绘制机器人实体
            _robot.Render(canvas, _offsetX, _offsetY, _scale, BaseCellPixels, CellSizeMeters);

            // 7. 绘制视口 HUD 提示面板（左上角半透明圆角卡片）
            DrawViewportHud(canvas);

            // 8. 绘制实时状态监测 HUD（左下角紧凑半透明圆角卡片，与标题卡片同风格）
            DrawStatusMonitorHud(canvas, width, height);
        }

        private void DrawPlannedPath(SKCanvas canvas, float cellPixels)
        {
            // 在地图上绘制从机器人当前位置到目标点的规划路径轨迹线。
            // cellPixels: 当前缩放比例下单格网格的像素宽度。
            if (_robot.CurrentPath == null || _robot.CurrentPath.Count == 0)
            {
                return;
            }

            using (var pathLine = new SKPath())
            using (var linePaint = new SKPaint
            {
                Color = new SKColor(0, 160, 240, 180),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(2.5f, 3.5f * _scale),
                StrokeCap = SKStrokeCap.Round,
                StrokeJoin = SKStrokeJoin.Round,
                IsAntialias = true
            })
            using (var dotPaint = new SKPaint
            {
                Color = new SKColor(0, 130, 210, 220),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            {
                float pixelsPerMeter = (BaseCellPixels * _scale) / CellSizeMeters;
                float robotScreenX = _offsetX + _robot.PositionX * pixelsPerMeter;
                float robotScreenY = _offsetY + _robot.PositionY * pixelsPerMeter;
                float dotRadius = Math.Max(2f, 3f * _scale);

                Point firstPt = _robot.CurrentPath[0];
                float firstPx = _offsetX + (firstPt.X + 0.5f) * cellPixels;
                float firstPy = _offsetY + (firstPt.Y + 0.5f) * cellPixels;

                // 仅当车身物理中心与前方第一个节点严格同轴（水平或垂直）时，从车身引出正交线段，杜绝任何斜线
                bool isCollinear = Math.Abs(robotScreenX - firstPx) < 1.0f || Math.Abs(robotScreenY - firstPy) < 1.0f;
                if (isCollinear)
                {
                    pathLine.MoveTo(robotScreenX, robotScreenY);
                    pathLine.LineTo(firstPx, firstPy);
                }
                else
                {
                    pathLine.MoveTo(firstPx, firstPy);
                }

                canvas.DrawCircle(firstPx, firstPy, dotRadius, dotPaint);

                for (int i = 1; i < _robot.CurrentPath.Count; i++)
                {
                    Point pt = _robot.CurrentPath[i];
                    float px = _offsetX + (pt.X + 0.5f) * cellPixels;
                    float py = _offsetY + (pt.Y + 0.5f) * cellPixels;
                    pathLine.LineTo(px, py);
                    canvas.DrawCircle(px, py, dotRadius, dotPaint);
                }

                canvas.DrawPath(pathLine, linePaint);
            }
        }

        private void DrawTargetMarker(SKCanvas canvas, float cellPixels)
        {
            // 在选定的目标网格上绘制高亮底衬与同心圆靶心标记
            if (!_selectedTarget.HasValue)
            {
                return;
            }

            Point target = _selectedTarget.Value;
            float cellX = _offsetX + target.X * cellPixels;
            float cellY = _offsetY + target.Y * cellPixels;
            float centerX = cellX + cellPixels * 0.5f;
            float centerY = cellY + cellPixels * 0.5f;

            using (var bgPaint = new SKPaint
            {
                Color = new SKColor(235, 60, 60, 50),
                Style = SKPaintStyle.Fill
            })
            using (var borderPaint = new SKPaint
            {
                Color = new SKColor(225, 40, 40),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(1.5f, 2f * _scale),
                IsAntialias = true
            })
            {
                canvas.DrawRect(cellX, cellY, cellPixels, cellPixels, bgPaint);
                canvas.DrawRect(cellX, cellY, cellPixels, cellPixels, borderPaint);
            }

            float outerRadius = cellPixels * 0.35f;
            float innerRadius = cellPixels * 0.18f;
            float centerDotRadius = Math.Max(1.5f, cellPixels * 0.08f);

            using (var ringPaint = new SKPaint
            {
                Color = new SKColor(220, 30, 30),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(1f, 1.8f * _scale),
                IsAntialias = true
            })
            using (var centerDotPaint = new SKPaint
            {
                Color = new SKColor(220, 30, 30),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            {
                canvas.DrawCircle(centerX, centerY, outerRadius, ringPaint);
                canvas.DrawCircle(centerX, centerY, innerRadius, ringPaint);
                canvas.DrawCircle(centerX, centerY, centerDotRadius, centerDotPaint);
            }
        }

        private void DrawViewportHud(SKCanvas canvas)
        {
            // 在左上角使用 SkiaSharp 绘制圆角 HUD 视口参数说明卡片。
            SKRect hudRect = new SKRect(12f, 12f, 320f, 116f);

            using (var panelPaint = new SKPaint
            {
                Color = new SKColor(255, 255, 255, 235),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            using (var strokePaint = new SKPaint
            {
                Color = new SKColor(190, 200, 210),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1f,
                IsAntialias = true
            })
            using (var textPaint = new SKPaint
            {
                Color = new SKColor(40, 50, 60),
                TextSize = 12f,
                IsAntialias = true,
                Typeface = SKTypeface.FromFamilyName("Microsoft YaHei")
            })
            {
                canvas.DrawRoundRect(hudRect, 6f, 6f, panelPaint);
                canvas.DrawRoundRect(hudRect, 6f, 6f, strokePaint);

                canvas.DrawText("30 x 30 仓储栅格地图 (SkiaSharp 渲染)", 20f, 32f, textPaint);
                canvas.DrawText("单格尺寸: 0.55m | 机器人离散中心对齐", 20f, 52f, textPaint);
                canvas.DrawText("视口操作: 鼠标左键拖拽平移 | 滚轮缩放", 20f, 72f, textPaint);
                canvas.DrawText("目标设定: 单击网格可随时设定或更改目标", 20f, 92f, textPaint);
                string fsmHud = "状态机: 闲置待命 (可改参数与算法)";
                switch (_robot.NavState)
                {
                    case RobotNavState.ManualMoving:
                        fsmHud = "状态机: 手动移动中 (键盘/按钮控制)";
                        break;
                    case RobotNavState.AutoMoving:
                        fsmHud = "状态机: 自动移动中 (算法锁定,可随时改目标)";
                        break;
                    case RobotNavState.Braking:
                        fsmHud = "状态机: 减速制动中 (平滑刹停在安全栅格)";
                        break;
                }
                canvas.DrawText(fsmHud, 20f, 112f, textPaint);
            }
        }

        private void DrawStatusMonitorHud(SKCanvas canvas, int width, int height)
        {
            // 在左下角使用 SkiaSharp 绘制紧凑美观的半透明圆角状态数据表格。
            float cardWidth = 320f;
            float cardHeight = 176f;
            float cardX = 12f;
            float cardY = height - cardHeight - 12f;

            if (cardY < 135f)
            {
                cardY = 135f;
            }

            SKRect hudRect = new SKRect(cardX, cardY, cardX + cardWidth, cardY + cardHeight);

            using (var panelPaint = new SKPaint
            {
                Color = new SKColor(255, 255, 255, 235),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            using (var borderPaint = new SKPaint
            {
                Color = new SKColor(190, 200, 210),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1f,
                IsAntialias = true
            })
            using (var titlePaint = new SKPaint
            {
                Color = new SKColor(25, 40, 55),
                TextSize = 12.5f,
                IsAntialias = true,
                Typeface = SKTypeface.FromFamilyName("Microsoft YaHei", SKFontStyle.Bold)
            })
            using (var keyPaint = new SKPaint
            {
                Color = new SKColor(60, 75, 95),
                TextSize = 10.8f,
                IsAntialias = true,
                Typeface = SKTypeface.FromFamilyName("Microsoft YaHei", SKFontStyle.Bold)
            })
            using (var valPaint = new SKPaint
            {
                Color = new SKColor(15, 23, 42),
                TextSize = 10.5f,
                IsAntialias = true,
                Typeface = SKTypeface.FromFamilyName("Microsoft YaHei")
            })
            using (var gridLinePaint = new SKPaint
            {
                Color = new SKColor(215, 222, 230, 220),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1f,
                IsAntialias = false
            })
            using (var headerBgPaint = new SKPaint
            {
                Color = new SKColor(235, 241, 246, 230),
                Style = SKPaintStyle.Fill
            })
            using (var keyColBgPaint = new SKPaint
            {
                Color = new SKColor(245, 247, 250, 190),
                Style = SKPaintStyle.Fill
            })
            using (var zebraRowPaint = new SKPaint
            {
                Color = new SKColor(248, 250, 252, 140),
                Style = SKPaintStyle.Fill
            })
            {
                // 1. 卡片外框与半透明圆角背景
                canvas.DrawRoundRect(hudRect, 6f, 6f, panelPaint);
                canvas.DrawRoundRect(hudRect, 6f, 6f, borderPaint);

                // 2. 卡片标题
                canvas.DrawText("机器人实时状态监测", cardX + 10f, cardY + 19f, titlePaint);

                // 3. 内嵌表格几何尺寸与坐标划分 (4 列双键值对表格，紧凑饱满)
                float tableX = cardX + 8f;
                float tableY = cardY + 27f;
                float tableW = cardWidth - 16f; // 304f
                float colKeyW = 46f;
                float colValW = 106f;
                float rowH = 20f;

                // 准备要显示的 6 行表格数据
                string fsmText = "闲置待命";
                switch (_robot.NavState)
                {
                    case RobotNavState.ManualMoving:
                        fsmText = "手动移动";
                        break;
                    case RobotNavState.AutoMoving:
                        fsmText = "自动移动";
                        break;
                    case RobotNavState.Braking:
                        fsmText = "减速制动";
                        break;
                    default:
                        fsmText = "闲置待命";
                        break;
                }
                string algoText = (_robot.CurrentAlgorithm == PathAlgorithm.Dijkstra) ? "Dijkstra" : "A*(曼哈顿)";

                string targetStr = _selectedTarget.HasValue ? ("[" + _selectedTarget.Value.X + "," + _selectedTarget.Value.Y + "]") : "未设定";
                int steps = _robot.CurrentPath != null ? _robot.CurrentPath.Count : 0;
                string targetVal = _selectedTarget.HasValue ? (targetStr + " 剩" + steps + "格") : "未设定";

                string orientStr = "上 ";
                switch (_robot.Orientation)
                {
                    case RobotOrientation.Right: orientStr = "右 "; break;
                    case RobotOrientation.Down: orientStr = "下 "; break;
                    case RobotOrientation.Left: orientStr = "左 "; break;
                }
                orientStr += _robot.CurrentAngle.ToString("0") + "°";

                string gridPos = "[" + _robot.GridX + ", " + _robot.GridY + "]";
                string physPos = "(" + _robot.PositionX.ToString("0.0") + "," + _robot.PositionY.ToString("0.0") + ")m";

                string actionText = "静止";
                if (_robot.ActionState == RobotActionState.Moving) actionText = "平移中";
                else if (_robot.ActionState == RobotActionState.Turning) actionText = "自转中";

                string linSpeed = _robot.CurrentSpeed.ToString("0.00") + "m/s";
                string linAccel = _robot.Acceleration.ToString("0.0") + "m/s²";

                float curOmega = (_robot.ActionState == RobotActionState.Turning) ? _robot.CurrentAngularVelocity : 0f;
                string angSpeed = curOmega.ToString("0") + "°/s";
                string angAccel = _robot.AngularAcceleration.ToString("0") + "°/s²";

                string[,] tableRows = new string[7, 4]
                {
                    { "指标", "状态/数值", "指标", "状态/数值" },
                    { "状态", fsmText, "算法", algoText },
                    { "目标", targetVal, "朝向", orientStr },
                    { "网格", gridPos, "物理", physPos },
                    { "动作", actionText, "线速", linSpeed },
                    { "加度", linAccel, "角速", angSpeed },
                    { "角加", angAccel, "底盘", "差动(0.38m)" }
                };

                // 4. 逐行渲染表格单元格与背景
                for (int r = 0; r < 7; r++)
                {
                    float yTop = tableY + r * rowH;
                    float yBottom = yTop + rowH;
                    float textBaseline = yTop + 14f;

                    if (r == 0)
                    {
                        // 表头行全宽背景
                        canvas.DrawRect(tableX, yTop, tableW, rowH, headerBgPaint);
                    }
                    else
                    {
                        // 数据行斑马纹背景
                        if (r % 2 == 1)
                        {
                            canvas.DrawRect(tableX, yTop, tableW, rowH, zebraRowPaint);
                        }

                        // 属性名称列（Key 列）专属冷灰底色
                        canvas.DrawRect(tableX, yTop, colKeyW, rowH, keyColBgPaint);
                        canvas.DrawRect(tableX + colKeyW + colValW, yTop, colKeyW, rowH, keyColBgPaint);
                    }

                    // 绘制单元格文本
                    SKPaint pKey = keyPaint;
                    SKPaint pVal = (r == 0) ? keyPaint : valPaint;

                    // Col 0: 参数1 (居中对齐感)
                    canvas.DrawText(tableRows[r, 0], tableX + 6f, textBaseline, pKey);
                    // Col 1: 数值1
                    canvas.DrawText(tableRows[r, 1], tableX + colKeyW + 5f, textBaseline, pVal);
                    // Col 2: 参数2
                    canvas.DrawText(tableRows[r, 2], tableX + colKeyW + colValW + 6f, textBaseline, pKey);
                    // Col 3: 数值2
                    canvas.DrawText(tableRows[r, 3], tableX + colKeyW + colValW + colKeyW + 5f, textBaseline, pVal);

                    // 绘制行底水平分割线
                    canvas.DrawLine(tableX, yBottom, tableX + tableW, yBottom, gridLinePaint);
                }

                // 5. 绘制表格外框与垂直分割线
                canvas.DrawRect(tableX, tableY, tableW, rowH * 7f, gridLinePaint);
                canvas.DrawLine(tableX + colKeyW, tableY, tableX + colKeyW, tableY + rowH * 7f, gridLinePaint);
                canvas.DrawLine(tableX + colKeyW + colValW, tableY, tableX + colKeyW + colValW, tableY + rowH * 7f, gridLinePaint);
                canvas.DrawLine(tableX + colKeyW + colValW + colKeyW, tableY, tableX + colKeyW + colValW + colKeyW, tableY + rowH * 7f, gridLinePaint);
            }
        }

        private void OnCanvasMouseDown(object sender, MouseEventArgs e)
        {
            // 响应鼠标按下并发起地图视口平移或记录选点初始坐标
            // sender: 触发事件的对象源
            // e: 鼠标事件数据
            if (e.Button == MouseButtons.Left || e.Button == MouseButtons.Right)
            {
                _isPanning = (e.Button == MouseButtons.Left);
                _lastMousePosition = e.Location;
                _mousePressLocation = e.Location;
                _hasDraggedMouse = false;
                if (e.Button == MouseButtons.Left)
                {
                    _skiaCanvas.Cursor = Cursors.SizeAll;
                    _skiaCanvas.Capture = true;
                }
            }
        }

        private void OnCanvasMouseMove(object sender, MouseEventArgs e)
        {
            // 响应鼠标拖拽并更新视口偏移像素
            if (_isPanning)
            {
                int dx = e.X - _lastMousePosition.X;
                int dy = e.Y - _lastMousePosition.Y;
                _offsetX += dx;
                _offsetY += dy;
                _lastMousePosition = e.Location;

                int totalDx = e.X - _mousePressLocation.X;
                int totalDy = e.Y - _mousePressLocation.Y;
                if (totalDx * totalDx + totalDy * totalDy > 16)
                {
                    _hasDraggedMouse = true;
                }

                _skiaCanvas.Invalidate();
            }
        }

        private void OnCanvasMouseUp(object sender, MouseEventArgs e)
        {
            // 响应鼠标按键释放并识别非拖拽点击以在地图中设置目标点
            if (_isPanning)
            {
                _isPanning = false;
                _skiaCanvas.Cursor = Cursors.Default;
                _skiaCanvas.Capture = false;
            }

            if (!_hasDraggedMouse)
            {
                // 排除左上角标题卡片区域和左下角状态卡片区域，避免点击卡片误设置底层网格目标点
                if ((e.X <= 340 && e.Y <= 135) || (e.X <= 400 && e.Y >= ClientRectangle.Height - 195))
                {
                    return;
                }

                float cellPixels = BaseCellPixels * _scale;
                int clickedX = (int)Math.Floor((e.X - _offsetX) / cellPixels);
                int clickedY = (int)Math.Floor((e.Y - _offsetY) / cellPixels);
                if (clickedX >= 0 && clickedX < GridColumns && clickedY >= 0 && clickedY < GridRows)
                {
                    Point clickedPoint = new Point(clickedX, clickedY);

                    // 再次点击同一个目标点即可取消目标点，效果等同于停止按钮，并清除红色标记
                    if (_selectedTarget.HasValue && _selectedTarget.Value == clickedPoint)
                    {
                        HandleCancelNavigation();
                    }
                    else
                    {
                        SetSelectedTarget(clickedPoint);
                    }
                }
            }
        }

        private void OnCanvasMouseWheel(object sender, MouseEventArgs e)
        {
            // 响应鼠标滚轮并以当前指针所在位置为中心缩放地图
            float requestedScale = e.Delta > 0 ? _scale * 1.1f : _scale / 1.1f;
            float newScale = Math.Max(MinimumScale, Math.Min(MaximumScale, requestedScale));

            if (Math.Abs(newScale - _scale) < 0.0001f)
            {
                return;
            }

            float worldX = (e.X - _offsetX) / _scale;
            float worldY = (e.Y - _offsetY) / _scale;
            _offsetX = e.X - worldX * newScale;
            _offsetY = e.Y - worldY * newScale;
            _scale = newScale;
            _skiaCanvas.Invalidate();
        }

        protected override void Dispose(bool disposing)
        {
            // 释放仿真定时器资源
            if (disposing)
            {
                if (_simulationTimer != null)
                {
                    _simulationTimer.Stop();
                    _simulationTimer.Dispose();
                }
            }
            base.Dispose(disposing);
        }
    }
}
