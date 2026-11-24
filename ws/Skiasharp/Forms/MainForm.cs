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
    /// 仿真运行模式枚举。
    /// </summary>
    public enum SimulationMode
    {
        /// <summary>
        /// 单机移动模式（用户手动指定目标点移动或键盘控制）。
        /// </summary>
        SingleRobot = 0,

        /// <summary>
        /// 多机器人调度模式（自动多机协同调度）。
        /// </summary>
        MultiRobot = 1
    }

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
        private SimulationMode _currentMode = SimulationMode.SingleRobot;

        private readonly Robot _robot;
        private readonly List<Robot> _multiRobots = new List<Robot>();
        private readonly List<Point> _multiTargets = new List<Point>();
        private readonly Random _random = new Random();

        private readonly Timer _simulationTimer;
        private readonly Stopwatch _stopwatch;
        private static SKTypeface _yaHeiTypeface;
        private static SKTypeface _yaHeiBoldTypeface;
        private static SKTypeface YaHeiTypeface => _yaHeiTypeface ??= SKTypeface.FromFamilyName("Microsoft YaHei");
        private static SKTypeface YaHeiBoldTypeface => _yaHeiBoldTypeface ??= SKTypeface.FromFamilyName("Microsoft YaHei", SKFontStyle.Bold);

        private double _lastElapsedSeconds;

        private SkiaCanvasControl _skiaCanvas;
        private Panel _controlPanel;

        private Panel _pnlSingleMode;
        private Panel _pnlMultiMode;
        private Panel _pnlCommonParams;

        private ComboBox _cmbMode;
        private ComboBox _cmbRobotCount;
        private Button _btnRandomizeRobots;
        private Label _lblMultiInfo;
        private ComboBox _cmbMultiAlgorithm;
        private Panel _pnlWhcaParams;
        private NumericUpDown _numWhcaK;
        private Panel _pnlEcbsParams;
        private NumericUpDown _numEcbsW;
        private Button _btnStartMultiNav;
        private Button _btnStopMultiNav;

        private bool _isMultiNavigating;
        private readonly Stopwatch _multiNavStopwatch = new Stopwatch();
        private float _multiNavTimeoutSeconds = 30f;
        private double _lastMultiProgressTime;
        private int _lastRemainingTotalSteps;
        private int _collisionCount = 0;
        private long _multiPlanElapsedMs = -1;
        private readonly HashSet<long> _activeCollisionPairs = new HashSet<long>();

        // 多机调度关键性能指标统计 (总任务完成时间、到达率、平均等待时间)
        private double _totalMultiWaitTime = 0.0;
        private double _finalMultiNavDuration = 0.0;

        // 视口 HUD 卡片与状态监测 HUD 卡片最小化状态与交互矩形
        private bool _isViewportHudMinimized = false;
        private bool _isStatusHudMinimized = false;
        private SKRect _viewportHudRect;
        private SKRect _statusHudRect;
        private SKRect _viewportMinimizeBtnRect;
        private SKRect _statusMinimizeBtnRect;

        // 工业级死锁、堵塞周期性智能诊断与自主恢复系统
        private readonly DeadlockManager _deadlockManager = new DeadlockManager();
        private double _lastDeadlockCheckTime = 0.0;

        // 右侧参数与操作 GUI 控制面板最小化/折叠状态管理
        private bool _isGuiPanelMinimized = false;
        private const int ControlPanelNormalWidth = 290;
        private const int ControlPanelMinimizedWidth = 36;
        private Panel _pnlMinimizedGuiBar;
        private Button _btnMinimizeGuiPanel;
        private Button _btnExpandGuiPanel;

        // 多机受控区域动态资源锁定与时空预约闭环管理系统 (网格顶点锁与无向边互斥锁)
        private readonly Dictionary<Point, int> _lockedZones = new Dictionary<Point, int>();
        private readonly Dictionary<int, HashSet<Point>> _robotClaimedZones = new Dictionary<int, HashSet<Point>>();
        private readonly Dictionary<long, int> _lockedEdges = new Dictionary<long, int>();
        private readonly Dictionary<int, HashSet<long>> _robotClaimedEdges = new Dictionary<int, HashSet<long>>();
        private readonly List<List<TimedPathPoint>> _agentTimedSchedules = new List<List<TimedPathPoint>>();
        private readonly double[] _agentTimeOffsets = new double[256];
        private readonly Point[] _robotLastOccupiedGrid = new Point[256];

        private static long GetUndirectedEdgeKey(Point u, Point v)
        {
            long k1 = ((long)u.X << 16) | (uint)(u.Y & 0xFFFF);
            long k2 = ((long)v.X << 16) | (uint)(v.Y & 0xFFFF);
            if (k1 > k2) { long t = k1; k1 = k2; k2 = t; }
            return (k1 << 32) | (k2 & 0xFFFFFFFFL);
        }

        private ComboBox _cmbAlgorithm;
        private Label _lblTargetInfo;
        private Button _btnStartNavigation;
        private Button _btnCancelNavigation;
        private CheckBox _chkShowPath;
        private CheckBox _chkShowTargetLines;

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
            MinimizeBox = true;
            MaximizeBox = true;

            _robot = new Robot(14, 14, CellSizeMeters, RobotOrientation.Up);
            _robot.NavStateChanged += OnRobotNavStateChanged;
            _robot.TargetReached += OnRobotTargetReached;

            InitializeComponents();

            _stopwatch = Stopwatch.StartNew();
            _lastElapsedSeconds = _stopwatch.Elapsed.TotalSeconds;

            _simulationTimer = new Timer();
            _simulationTimer.Interval = 16;
            _simulationTimer.Tick += OnSimulationTick;
        }

        protected override void OnShown(EventArgs e)
        {
            // 窗体完成呈现后启动高精度物理仿真定时器，杜绝初始化阶段重绘卡顿
            base.OnShown(e);
            if (_simulationTimer != null && !_simulationTimer.Enabled)
            {
                _lastElapsedSeconds = _stopwatch.Elapsed.TotalSeconds;
                _simulationTimer.Start();
            }
        }

        private void InitializeComponents()
        {
            // 创建并布局 Skia 画布与右侧控制交互面板 (使用 SuspendLayout 优化冷启动耗时)
            SuspendLayout();

            _controlPanel = new Panel();
            _controlPanel.SuspendLayout();
            _controlPanel.Dock = DockStyle.Right;
            _controlPanel.Width = 290;
            _controlPanel.BackColor = Color.White;
            _controlPanel.BorderStyle = BorderStyle.FixedSingle;
            _controlPanel.AutoScroll = true;

            BuildControlPanelContent();

            _controlPanel.ResumeLayout(false);
            _controlPanel.PerformLayout();

            _skiaCanvas = new SkiaCanvasControl();
            _skiaCanvas.Dock = DockStyle.Fill;
            _skiaCanvas.PaintSurface += OnPaintSkiaSurface;
            _skiaCanvas.MouseDown += OnCanvasMouseDown;
            _skiaCanvas.MouseMove += OnCanvasMouseMove;
            _skiaCanvas.MouseUp += OnCanvasMouseUp;
            _skiaCanvas.MouseWheel += OnCanvasMouseWheel;

            Controls.Add(_skiaCanvas);
            Controls.Add(_controlPanel);

            ResumeLayout(false);
            PerformLayout();
        }

        private void BuildControlPanelContent()
        {
            // 0. 构建右侧控制面板最小化折叠状态下的立式竖条
            _pnlMinimizedGuiBar = new Panel();
            _pnlMinimizedGuiBar.Dock = DockStyle.Fill;
            _pnlMinimizedGuiBar.BackColor = Color.FromArgb(246, 248, 251);
            _pnlMinimizedGuiBar.Visible = false;
            _pnlMinimizedGuiBar.Cursor = Cursors.Hand;
            _pnlMinimizedGuiBar.Click += delegate { SetGuiPanelMinimized(false); };
            _pnlMinimizedGuiBar.Paint += OnMinimizedGuiBarPaint;

            _btnExpandGuiPanel = new Button();
            _btnExpandGuiPanel.Text = "◀";
            _btnExpandGuiPanel.Font = new Font("Microsoft YaHei", 8.5f, FontStyle.Bold);
            _btnExpandGuiPanel.Location = new Point(6, 8);
            _btnExpandGuiPanel.Size = new Size(24, 24);
            _btnExpandGuiPanel.FlatStyle = FlatStyle.Flat;
            _btnExpandGuiPanel.FlatAppearance.BorderSize = 1;
            _btnExpandGuiPanel.FlatAppearance.BorderColor = Color.FromArgb(180, 195, 210);
            _btnExpandGuiPanel.BackColor = Color.FromArgb(235, 242, 250);
            _btnExpandGuiPanel.ForeColor = Color.FromArgb(40, 60, 85);
            _btnExpandGuiPanel.Cursor = Cursors.Hand;
            _btnExpandGuiPanel.Click += delegate { SetGuiPanelMinimized(false); };

            ToolTip toolTip = new ToolTip();
            toolTip.SetToolTip(_btnExpandGuiPanel, "展开参数调节与控制面板");
            toolTip.SetToolTip(_pnlMinimizedGuiBar, "点击展开参数调节与控制面板");

            _pnlMinimizedGuiBar.Controls.Add(_btnExpandGuiPanel);
            _controlPanel.Controls.Add(_pnlMinimizedGuiBar);

            // 构造右侧面板内的所有参数调节控件、动作按钮和状态卡片
            int currentY = 16;

            Label lblTitle = new Label();
            lblTitle.Text = "参数调节与寻路控制";
            lblTitle.Font = new Font("Microsoft YaHei", 10.5f, FontStyle.Bold);
            lblTitle.ForeColor = Color.FromArgb(30, 45, 60);
            lblTitle.Location = new Point(14, currentY);
            lblTitle.Size = new Size(224, 24);
            _controlPanel.Controls.Add(lblTitle);

            _btnMinimizeGuiPanel = new Button();
            _btnMinimizeGuiPanel.Text = "▶";
            _btnMinimizeGuiPanel.Font = new Font("Microsoft YaHei", 8f, FontStyle.Bold);
            _btnMinimizeGuiPanel.Location = new Point(244, currentY - 2);
            _btnMinimizeGuiPanel.Size = new Size(26, 24);
            _btnMinimizeGuiPanel.FlatStyle = FlatStyle.Flat;
            _btnMinimizeGuiPanel.FlatAppearance.BorderSize = 1;
            _btnMinimizeGuiPanel.FlatAppearance.BorderColor = Color.FromArgb(195, 205, 215);
            _btnMinimizeGuiPanel.BackColor = Color.FromArgb(242, 246, 250);
            _btnMinimizeGuiPanel.ForeColor = Color.FromArgb(50, 70, 95);
            _btnMinimizeGuiPanel.Cursor = Cursors.Hand;
            _btnMinimizeGuiPanel.Click += delegate { SetGuiPanelMinimized(true); };
            toolTip.SetToolTip(_btnMinimizeGuiPanel, "最小化收起控制面板 (最大化视口)");
            _controlPanel.Controls.Add(_btnMinimizeGuiPanel);
            currentY += 34;

            _controlPanel.Controls.Add(CreateHorizontalSeparator(currentY));
            currentY += 14;

            // 0. 仿真运行模式选择
            Label lblModeTitle = new Label();
            lblModeTitle.Text = "运行模式选择:";
            lblModeTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblModeTitle.Location = new Point(14, currentY);
            lblModeTitle.Size = new Size(250, 22);
            _controlPanel.Controls.Add(lblModeTitle);
            currentY += 24;

            _cmbMode = new ComboBox();
            _cmbMode.DropDownStyle = ComboBoxStyle.DropDownList;
            _cmbMode.Font = new Font("Microsoft YaHei", 9f);
            _cmbMode.Location = new Point(14, currentY);
            _cmbMode.Size = new Size(250, 26);
            _cmbMode.Items.Add("单机移动模式 (手动指定目标)");
            _cmbMode.Items.Add("多机调度模式 (自动协同调度)");
            _cmbMode.SelectedIndex = 0;
            _cmbMode.SelectedIndexChanged += OnModeSelectedIndexChanged;
            _controlPanel.Controls.Add(_cmbMode);
            currentY += 36;

            _controlPanel.Controls.Add(CreateHorizontalSeparator(currentY));
            currentY += 14;

            int modePanelTop = currentY;

            // 1. 单机模式专用控制面板
            _pnlSingleMode = new Panel();
            _pnlSingleMode.SuspendLayout();
            _pnlSingleMode.Location = new Point(0, modePanelTop);
            _pnlSingleMode.Width = 285;

            int sY = 0;

            // 1.1 单机寻路规划算法选项
            Label lblAlgoTitle = new Label();
            lblAlgoTitle.Text = "寻路规划算法 (单机):";
            lblAlgoTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblAlgoTitle.Location = new Point(14, sY);
            lblAlgoTitle.Size = new Size(250, 22);
            _pnlSingleMode.Controls.Add(lblAlgoTitle);
            sY += 24;

            _cmbAlgorithm = new ComboBox();
            _cmbAlgorithm.DropDownStyle = ComboBoxStyle.DropDownList;
            _cmbAlgorithm.Font = new Font("Microsoft YaHei", 9f);
            _cmbAlgorithm.Location = new Point(14, sY);
            _cmbAlgorithm.Size = new Size(250, 26);
            _cmbAlgorithm.Items.Add("A* 算法 (曼哈顿距离启发式)");
            _cmbAlgorithm.Items.Add("Dijkstra 算法 (无启发式)");
            _cmbAlgorithm.SelectedIndex = 0;
            _cmbAlgorithm.SelectedIndexChanged += OnAlgorithmSelectedIndexChanged;
            _pnlSingleMode.Controls.Add(_cmbAlgorithm);
            sY += 36;

            _pnlSingleMode.Controls.Add(CreateHorizontalSeparator(sY));
            sY += 14;

            // 1.2 目标点设置与自动移动
            Label lblTargetGroupTitle = new Label();
            lblTargetGroupTitle.Text = "目标网格 (点击地图设置):";
            lblTargetGroupTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblTargetGroupTitle.Location = new Point(14, sY);
            lblTargetGroupTitle.Size = new Size(250, 22);
            _pnlSingleMode.Controls.Add(lblTargetGroupTitle);
            sY += 24;

            _lblTargetInfo = new Label();
            _lblTargetInfo.Text = "未设置目标 (在地图中单击网格)";
            _lblTargetInfo.Font = new Font("Microsoft YaHei", 8.5f);
            _lblTargetInfo.ForeColor = Color.FromArgb(120, 130, 140);
            _lblTargetInfo.Location = new Point(14, sY);
            _lblTargetInfo.Size = new Size(250, 24);
            _pnlSingleMode.Controls.Add(_lblTargetInfo);
            sY += 28;

            _btnStartNavigation = new Button();
            _btnStartNavigation.Text = "确定目标并自动移动";
            _btnStartNavigation.Location = new Point(14, sY);
            _btnStartNavigation.Size = new Size(250, 32);
            _btnStartNavigation.BackColor = Color.FromArgb(230, 245, 255);
            _btnStartNavigation.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            _btnStartNavigation.ForeColor = Color.FromArgb(0, 100, 200);
            _btnStartNavigation.Click += delegate { HandleStartNavigation(); };
            _pnlSingleMode.Controls.Add(_btnStartNavigation);
            sY += 36;

            _btnCancelNavigation = new Button();
            _btnCancelNavigation.Text = "取消导航";
            _btnCancelNavigation.Location = new Point(14, sY);
            _btnCancelNavigation.Size = new Size(250, 28);
            _btnCancelNavigation.Font = new Font("Microsoft YaHei", 8.5f);
            _btnCancelNavigation.Click += delegate { HandleCancelNavigation(); };
            _pnlSingleMode.Controls.Add(_btnCancelNavigation);
            sY += 34;

            _pnlSingleMode.Controls.Add(CreateHorizontalSeparator(sY));
            sY += 14;

            Label lblActions = new Label();
            lblActions.Text = "手动控制 (或使用键盘):";
            lblActions.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblActions.Location = new Point(14, sY);
            lblActions.Size = new Size(250, 24);
            _pnlSingleMode.Controls.Add(lblActions);
            sY += 28;

            _btnMoveForward = new Button();
            _btnMoveForward.Text = "前进一格 [W / ↑]";
            _btnMoveForward.Location = new Point(14, sY);
            _btnMoveForward.Size = new Size(250, 32);
            _btnMoveForward.BackColor = Color.FromArgb(235, 245, 255);
            _btnMoveForward.Font = new Font("Microsoft YaHei", 9f);
            _btnMoveForward.Click += delegate { HandleMoveForward(); };
            _pnlSingleMode.Controls.Add(_btnMoveForward);
            sY += 38;

            _btnTurnLeft = new Button();
            _btnTurnLeft.Text = "左转 90° [A / ←]";
            _btnTurnLeft.Location = new Point(14, sY);
            _btnTurnLeft.Size = new Size(120, 32);
            _btnTurnLeft.Font = new Font("Microsoft YaHei", 9f);
            _btnTurnLeft.Click += delegate { HandleTurnLeft(); };
            _pnlSingleMode.Controls.Add(_btnTurnLeft);

            _btnTurnRight = new Button();
            _btnTurnRight.Text = "右转 90° [D / →]";
            _btnTurnRight.Location = new Point(144, sY);
            _btnTurnRight.Size = new Size(120, 32);
            _btnTurnRight.Font = new Font("Microsoft YaHei", 9f);
            _btnTurnRight.Click += delegate { HandleTurnRight(); };
            _pnlSingleMode.Controls.Add(_btnTurnRight);
            sY += 38;

            _btnResetPosition = new Button();
            _btnResetPosition.Text = "复位到中心 [空格]";
            _btnResetPosition.Location = new Point(14, sY);
            _btnResetPosition.Size = new Size(250, 30);
            _btnResetPosition.Font = new Font("Microsoft YaHei", 8.5f);
            _btnResetPosition.Click += delegate { HandleResetPosition(); };
            _pnlSingleMode.Controls.Add(_btnResetPosition);
            sY += 36;

            _pnlSingleMode.Height = sY;
            _pnlSingleMode.ResumeLayout(false);
            _controlPanel.Controls.Add(_pnlSingleMode);

            // 2. 多机模式专用控制面板 (无单机寻路算法选择)
            _pnlMultiMode = new Panel();
            _pnlMultiMode.SuspendLayout();
            _pnlMultiMode.Location = new Point(0, modePanelTop);
            _pnlMultiMode.Width = 285;
            _pnlMultiMode.Visible = false;

            int mY = 0;
            Label lblMultiTitle = new Label();
            lblMultiTitle.Text = "多机调度配置:";
            lblMultiTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblMultiTitle.Location = new Point(14, mY);
            lblMultiTitle.Size = new Size(250, 22);
            _pnlMultiMode.Controls.Add(lblMultiTitle);
            mY += 24;

            Label lblCountTitle = new Label();
            lblCountTitle.Text = "机器人与目标点数量:";
            lblCountTitle.Font = new Font("Microsoft YaHei", 8.5f);
            lblCountTitle.ForeColor = Color.FromArgb(70, 80, 95);
            lblCountTitle.Location = new Point(14, mY);
            lblCountTitle.Size = new Size(250, 20);
            _pnlMultiMode.Controls.Add(lblCountTitle);
            mY += 22;

            _cmbRobotCount = new ComboBox();
            _cmbRobotCount.DropDownStyle = ComboBoxStyle.DropDownList;
            _cmbRobotCount.Font = new Font("Microsoft YaHei", 9f);
            _cmbRobotCount.Location = new Point(14, mY);
            _cmbRobotCount.Size = new Size(250, 26);
            _cmbRobotCount.Items.AddRange(new object[] { 5, 10, 20, 50, 100, 200 });
            _cmbRobotCount.SelectedItem = 20;
            _cmbRobotCount.SelectedIndexChanged += OnRobotCountSelectedIndexChanged;
            _pnlMultiMode.Controls.Add(_cmbRobotCount);
            mY += 36;

            _btnRandomizeRobots = new Button();
            _btnRandomizeRobots.Text = "随机生成机群与对角目标点";
            _btnRandomizeRobots.Location = new Point(14, mY);
            _btnRandomizeRobots.Size = new Size(250, 32);
            _btnRandomizeRobots.BackColor = Color.FromArgb(245, 235, 255);
            _btnRandomizeRobots.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            _btnRandomizeRobots.ForeColor = Color.FromArgb(120, 50, 180);
            _btnRandomizeRobots.Click += delegate { HandleRandomizeRobots(); };
            _pnlMultiMode.Controls.Add(_btnRandomizeRobots);
            mY += 36;

            _chkShowTargetLines = new CheckBox();
            _chkShowTargetLines.Text = "开启目标配对连线";
            _chkShowTargetLines.Font = new Font("Microsoft YaHei", 9f);
            _chkShowTargetLines.Checked = true;
            _chkShowTargetLines.Location = new Point(14, mY);
            _chkShowTargetLines.Size = new Size(250, 24);
            _chkShowTargetLines.CheckedChanged += delegate { _skiaCanvas?.Invalidate(); };
            _pnlMultiMode.Controls.Add(_chkShowTargetLines);
            mY += 28;

            _pnlMultiMode.Controls.Add(CreateHorizontalSeparator(mY));
            mY += 12;

            Label lblMultiAlgoTitle = new Label();
            lblMultiAlgoTitle.Text = "多机协同调度算法:";
            lblMultiAlgoTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblMultiAlgoTitle.Location = new Point(14, mY);
            lblMultiAlgoTitle.Size = new Size(250, 20);
            _pnlMultiMode.Controls.Add(lblMultiAlgoTitle);
            mY += 22;

            _cmbMultiAlgorithm = new ComboBox();
            _cmbMultiAlgorithm.DropDownStyle = ComboBoxStyle.DropDownList;
            _cmbMultiAlgorithm.Font = new Font("Microsoft YaHei", 9f);
            _cmbMultiAlgorithm.Location = new Point(14, mY);
            _cmbMultiAlgorithm.Size = new Size(250, 26);
            _cmbMultiAlgorithm.Items.Add("CBS (基于冲突搜索)");
            _cmbMultiAlgorithm.Items.Add("ECBS (增强型CBS快速算法)");
            _cmbMultiAlgorithm.Items.Add("CA* (协同A*算法)");
            _cmbMultiAlgorithm.Items.Add("WHCA* (窗口协同A*算法)");
            _cmbMultiAlgorithm.SelectedIndex = 1;
            _cmbMultiAlgorithm.SelectedIndexChanged += OnMultiAlgorithmSelectedIndexChanged;
            _pnlMultiMode.Controls.Add(_cmbMultiAlgorithm);
            mY += 34;

            // 2.3 WHCA* 专属时间窗口 K 参数设置面板
            _pnlWhcaParams = new Panel();
            _pnlWhcaParams.Location = new Point(14, mY);
            _pnlWhcaParams.Size = new Size(250, 30);
            _pnlWhcaParams.Visible = false;

            Label lblWhcaK = new Label();
            lblWhcaK.Text = "时间窗口 K (步长):";
            lblWhcaK.Font = new Font("Microsoft YaHei", 8.5f, FontStyle.Bold);
            lblWhcaK.Location = new Point(0, 4);
            lblWhcaK.Size = new Size(120, 22);
            _pnlWhcaParams.Controls.Add(lblWhcaK);

            _numWhcaK = new NumericUpDown();
            _numWhcaK.Location = new Point(130, 2);
            _numWhcaK.Size = new Size(120, 24);
            _numWhcaK.Minimum = 4;
            _numWhcaK.Maximum = 64;
            _numWhcaK.Value = 18;
            _numWhcaK.Increment = 2;
            _pnlWhcaParams.Controls.Add(_numWhcaK);
            _pnlMultiMode.Controls.Add(_pnlWhcaParams);

            // 2.4 ECBS 专属次优解因子 w 参数设置面板
            _pnlEcbsParams = new Panel();
            _pnlEcbsParams.Location = new Point(14, mY);
            _pnlEcbsParams.Size = new Size(250, 30);
            _pnlEcbsParams.Visible = true;

            Label lblEcbsW = new Label();
            lblEcbsW.Text = "次优解因子 w:";
            lblEcbsW.Font = new Font("Microsoft YaHei", 8.5f, FontStyle.Bold);
            lblEcbsW.Location = new Point(0, 4);
            lblEcbsW.Size = new Size(120, 22);
            _pnlEcbsParams.Controls.Add(lblEcbsW);

            _numEcbsW = new NumericUpDown();
            _numEcbsW.Location = new Point(130, 2);
            _numEcbsW.Size = new Size(120, 24);
            _numEcbsW.DecimalPlaces = 2;
            _numEcbsW.Minimum = 1.05m;
            _numEcbsW.Maximum = 2.50m;
            _numEcbsW.Value = 1.30m;
            _numEcbsW.Increment = 0.05m;
            _pnlEcbsParams.Controls.Add(_numEcbsW);
            _pnlMultiMode.Controls.Add(_pnlEcbsParams);
            mY += 34;

            _btnStartMultiNav = new Button();
            _btnStartMultiNav.Text = "执行多机协同调度导航";
            _btnStartMultiNav.Location = new Point(14, mY);
            _btnStartMultiNav.Size = new Size(250, 32);
            _btnStartMultiNav.BackColor = Color.FromArgb(230, 245, 255);
            _btnStartMultiNav.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            _btnStartMultiNav.ForeColor = Color.FromArgb(0, 100, 200);
            _btnStartMultiNav.Click += delegate { HandleStartMultiNavigation(); };
            _pnlMultiMode.Controls.Add(_btnStartMultiNav);
            mY += 36;

            _btnStopMultiNav = new Button();
            _btnStopMultiNav.Text = "停止多机调度";
            _btnStopMultiNav.Location = new Point(14, mY);
            _btnStopMultiNav.Size = new Size(250, 28);
            _btnStopMultiNav.Font = new Font("Microsoft YaHei", 8.5f);
            _btnStopMultiNav.Click += delegate { HandleStopMultiNavigation(); };
            _pnlMultiMode.Controls.Add(_btnStopMultiNav);
            mY += 32;

            _lblMultiInfo = new Label();
            _lblMultiInfo.Text = "机群: 20台 | 目标: 20个 (就绪待命)";
            _lblMultiInfo.Font = new Font("Microsoft YaHei", 8.5f);
            _lblMultiInfo.ForeColor = Color.FromArgb(40, 140, 60);
            _lblMultiInfo.Location = new Point(14, mY);
            _lblMultiInfo.Size = new Size(250, 36);
            _pnlMultiMode.Controls.Add(_lblMultiInfo);
            mY += 40;

            _pnlMultiMode.Height = mY;
            _pnlMultiMode.ResumeLayout(false);
            _controlPanel.Controls.Add(_pnlMultiMode);

            // 3. 通用参数设置面板 (仅保留运动参数与显示设置)
            _pnlCommonParams = new Panel();
            _pnlCommonParams.SuspendLayout();
            _pnlCommonParams.Width = 285;

            int cY = 0;
            _pnlCommonParams.Controls.Add(CreateHorizontalSeparator(cY));
            cY += 14;

            _chkShowPath = new CheckBox();
            _chkShowPath.Text = "开启路径可视化轨迹";
            _chkShowPath.Font = new Font("Microsoft YaHei", 9f);
            _chkShowPath.Checked = true;
            _chkShowPath.Location = new Point(14, cY);
            _chkShowPath.Size = new Size(250, 24);
            _chkShowPath.CheckedChanged += delegate { _skiaCanvas?.Invalidate(); };
            _pnlCommonParams.Controls.Add(_chkShowPath);
            cY += 32;

            _pnlCommonParams.Controls.Add(CreateHorizontalSeparator(cY));
            cY += 14;

            // 最大速度调节
            Label lblSpeedTitle = new Label();
            lblSpeedTitle.Text = "最大速度 (m/s):";
            lblSpeedTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblSpeedTitle.Location = new Point(14, cY);
            lblSpeedTitle.Size = new Size(130, 24);
            _pnlCommonParams.Controls.Add(lblSpeedTitle);

            _numSpeed = new NumericUpDown();
            _numSpeed.Location = new Point(155, cY - 2);
            _numSpeed.Size = new Size(105, 24);
            _numSpeed.DecimalPlaces = 2;
            _numSpeed.Increment = 0.1m;
            _numSpeed.Minimum = 0.20m;
            _numSpeed.Maximum = 4.00m;
            _numSpeed.Value = 1.50m;
            _numSpeed.ValueChanged += OnNumSpeedValueChanged;
            _pnlCommonParams.Controls.Add(_numSpeed);
            cY += 32;

            _trackSpeed = new TrackBar();
            _trackSpeed.Location = new Point(10, cY);
            _trackSpeed.Size = new Size(255, 45);
            _trackSpeed.Minimum = 20;
            _trackSpeed.Maximum = 400;
            _trackSpeed.TickFrequency = 40;
            _trackSpeed.Value = 150;
            _trackSpeed.Scroll += OnTrackSpeedScroll;
            _pnlCommonParams.Controls.Add(_trackSpeed);
            cY += 54;

            // 加速度调节
            Label lblAccelTitle = new Label();
            lblAccelTitle.Text = "加速度 (m/s²):";
            lblAccelTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblAccelTitle.Location = new Point(14, cY);
            lblAccelTitle.Size = new Size(130, 24);
            _pnlCommonParams.Controls.Add(lblAccelTitle);

            _numAccel = new NumericUpDown();
            _numAccel.Location = new Point(155, cY - 2);
            _numAccel.Size = new Size(105, 24);
            _numAccel.DecimalPlaces = 2;
            _numAccel.Increment = 0.2m;
            _numAccel.Minimum = 0.20m;
            _numAccel.Maximum = 6.00m;
            _numAccel.Value = 2.00m;
            _numAccel.ValueChanged += OnNumAccelValueChanged;
            _pnlCommonParams.Controls.Add(_numAccel);
            cY += 32;

            _trackAccel = new TrackBar();
            _trackAccel.Location = new Point(10, cY);
            _trackAccel.Size = new Size(255, 45);
            _trackAccel.Minimum = 20;
            _trackAccel.Maximum = 600;
            _trackAccel.TickFrequency = 60;
            _trackAccel.Value = 200;
            _trackAccel.Scroll += OnTrackAccelScroll;
            _pnlCommonParams.Controls.Add(_trackAccel);
            cY += 54;

            _btnResetParams = new Button();
            _btnResetParams.Text = "恢复默认参数 (1.5m/s, 2.0m/s²)";
            _btnResetParams.Location = new Point(14, cY);
            _btnResetParams.Size = new Size(250, 30);
            _btnResetParams.Font = new Font("Microsoft YaHei", 8.5f);
            _btnResetParams.Click += delegate
            {
                _numSpeed.Value = 1.50m;
                _numAccel.Value = 2.00m;
            };
            _pnlCommonParams.Controls.Add(_btnResetParams);
            cY += 42;

            _pnlCommonParams.Height = cY;
            _pnlCommonParams.ResumeLayout(false);
            _controlPanel.Controls.Add(_pnlCommonParams);

            UpdateMultiAlgoParamsVisibility();
            UpdateModeControlsLayout();
        }

        private void OnMultiAlgorithmSelectedIndexChanged(object sender, EventArgs e)
        {
            // 响应多机调度算法切换并动态展示对应算法的专属参数面板 (WHCA* 窗口 K, ECBS 次优因子 w)
            UpdateMultiAlgoParamsVisibility();
            _skiaCanvas?.Invalidate();
        }

        private void UpdateMultiAlgoParamsVisibility()
        {
            // 根据选定的多机调度算法动态显隐专属参数面板，选择其他算法时完全不显示，自适应紧凑排版
            if (_cmbMultiAlgorithm == null || _pnlMultiMode == null)
            {
                return;
            }

            int selectedIndex = _cmbMultiAlgorithm.SelectedIndex;
            bool isEcbs = (selectedIndex == 1);
            bool isWhca = (selectedIndex == 3);

            if (_pnlEcbsParams != null) _pnlEcbsParams.Visible = isEcbs;
            if (_pnlWhcaParams != null) _pnlWhcaParams.Visible = isWhca;

            // 动态紧凑自适应调整下方按钮与说明的 Y 坐标
            int currentY = _cmbMultiAlgorithm.Bottom + 8;

            if (isEcbs && _pnlEcbsParams != null)
            {
                _pnlEcbsParams.Top = currentY;
                currentY = _pnlEcbsParams.Bottom + 8;
            }
            else if (isWhca && _pnlWhcaParams != null)
            {
                _pnlWhcaParams.Top = currentY;
                currentY = _pnlWhcaParams.Bottom + 8;
            }

            if (_btnStartMultiNav != null)
            {
                _btnStartMultiNav.Top = currentY;
                currentY = _btnStartMultiNav.Bottom + 6;
            }

            if (_btnStopMultiNav != null)
            {
                _btnStopMultiNav.Top = currentY;
                currentY = _btnStopMultiNav.Bottom + 8;
            }

            if (_lblMultiInfo != null)
            {
                _lblMultiInfo.Top = currentY;
                currentY = _lblMultiInfo.Bottom + 10;
            }

            _pnlMultiMode.Height = currentY;
        }

        private void UpdateModeControlsLayout()
        {
            // 依据当前模式动态调整单机与多机专有面板的显隐及通用参数区布局
            bool isSingle = (_currentMode == SimulationMode.SingleRobot);
            if (_pnlSingleMode != null) _pnlSingleMode.Visible = isSingle;
            if (_pnlMultiMode != null) _pnlMultiMode.Visible = !isSingle;

            if (_pnlCommonParams != null)
            {
                int nextY = isSingle
                    ? (_pnlSingleMode != null ? _pnlSingleMode.Bottom + 6 : 400)
                    : (_pnlMultiMode != null ? _pnlMultiMode.Bottom + 6 : 300);
                _pnlCommonParams.Top = nextY;
            }
        }

        private void SetGuiPanelMinimized(bool minimized)
        {
            _isGuiPanelMinimized = minimized;
            _controlPanel.SuspendLayout();

            if (_isGuiPanelMinimized)
            {
                _controlPanel.AutoScroll = false;
                foreach (Control ctrl in _controlPanel.Controls)
                {
                    if (ctrl != _pnlMinimizedGuiBar)
                    {
                        ctrl.Visible = false;
                    }
                }
                _pnlMinimizedGuiBar.Visible = true;
                _controlPanel.Width = ControlPanelMinimizedWidth;
                _pnlMinimizedGuiBar.Invalidate();
            }
            else
            {
                _pnlMinimizedGuiBar.Visible = false;
                _controlPanel.Width = ControlPanelNormalWidth;
                _controlPanel.AutoScroll = true;

                foreach (Control ctrl in _controlPanel.Controls)
                {
                    if (ctrl != _pnlMinimizedGuiBar)
                    {
                        ctrl.Visible = true;
                    }
                }

                UpdateModeControlsLayout();
            }

            _controlPanel.ResumeLayout(true);
            _skiaCanvas?.Invalidate();
        }

        private void OnMinimizedGuiBarPaint(object sender, PaintEventArgs e)
        {
            // 竖向自适应高保真绘制折叠栏标题，支持任意系统 DPI，确保 9 个汉字完整无遮挡且水平绝对居中
            e.Graphics.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;

            string title = "参数调节与寻路控制";
            using (var font = new Font("Microsoft YaHei", 8.5f, FontStyle.Bold))
            using (var brush = new SolidBrush(Color.FromArgb(60, 80, 105)))
            {
                float startY = 40f;
                float stepY = 21f;
                float panelW = _pnlMinimizedGuiBar.Width;

                for (int i = 0; i < title.Length; i++)
                {
                    string ch = title[i].ToString();
                    SizeF sz = e.Graphics.MeasureString(ch, font);
                    float x = (panelW - sz.Width) * 0.5f;
                    float y = startY + i * stepY;
                    e.Graphics.DrawString(ch, font, brush, x, y);
                }
            }
        }

        private void HandleStartMultiNavigation()
        {
            // 响应用户请求，使用选定多机协同算法执行全局路径规划并启动机群协同导航。
            if (_isMultiNavigating)
            {
                return;
            }

            if (_multiRobots == null || _multiRobots.Count == 0 || _multiTargets == null || _multiTargets.Count == 0)
            {
                MessageBox.Show(this, "请先在界面上生成机器人机群与目标点！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            // 获取用户当前选定的多机调度算法 (CBS, ECBS, CA*, WHCA*)
            MultiAgentAlgorithm selectedAlgorithm = (MultiAgentAlgorithm)_cmbMultiAlgorithm.SelectedIndex;
            string algoName = _cmbMultiAlgorithm.SelectedItem?.ToString() ?? "CBS";

            // 构造多机任务请求清单
            List<AgentTask> tasks = new List<AgentTask>(_multiRobots.Count);
            for (int i = 0; i < _multiRobots.Count; i++)
            {
                tasks.Add(new AgentTask(i, new Point(_multiRobots[i].GridX, _multiRobots[i].GridY), _multiTargets[i]));
            }

            _lblMultiInfo.Text = $"正在使用 {algoName} 进行无碰撞规划...";
            _lblMultiInfo.ForeColor = Color.FromArgb(0, 100, 200);
            Refresh();

            int whcaK = (_numWhcaK != null) ? (int)_numWhcaK.Value : 18;
            double ecbsW = (_numEcbsW != null) ? (double)_numEcbsW.Value : 1.30;

            // 调用统一的多智能体路径规划器 (带 4000ms 超时死锁保护与自定义算法参数)
            MultiAgentPlanResult planResult = MultiAgentPathPlanning.Plan(
                tasks,
                GridColumns,
                GridRows,
                selectedAlgorithm,
                timeoutMs: 4000,
                whcaWindowK: whcaK,
                ecbsSuboptimality: ecbsW);

            _multiPlanElapsedMs = planResult.ElapsedMilliseconds;

            // 若算法检测到死锁或超时无法求解
            if (!planResult.Success || planResult.IsTimeout)
            {
                _lblMultiInfo.Text = "任务失败: 检测到死锁或路径不可解，调度超时！";
                _lblMultiInfo.ForeColor = Color.FromArgb(220, 40, 40);
                MessageBox.Show(
                    this,
                    $"多机协同调度任务失败！\n\n原因: {planResult.Message}\n路径求解时间: {planResult.ElapsedMilliseconds} ms\n\n请尝试更换其它多机算法 (如 ECBS/CA*) 或重新随机生成分布。",
                    "调度失败 (死锁超时)",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Warning);
                _skiaCanvas?.Invalidate();
                return;
            }

            // 规划成功，依次将各智能体的正交无冲突时空路径赋给机器人
            int maxPathSteps = 0;
            int totalSteps = 0;

            // 流程步骤 1：规划未来路径并提交保守时间预约与初始受控区域锁定
            _lockedZones.Clear();
            _robotClaimedZones.Clear();
            _lockedEdges.Clear();
            _robotClaimedEdges.Clear();
            _agentTimedSchedules.Clear();
            if (planResult.TimedSchedules != null)
            {
                _agentTimedSchedules.AddRange(planResult.TimedSchedules);
            }

            for (int i = 0; i < _multiRobots.Count; i++)
            {
                _robotClaimedZones[i] = new HashSet<Point>();
                _robotClaimedEdges[i] = new HashSet<long>();
                _agentTimeOffsets[i] = 0.0;
                Point startGrid = new Point(_multiRobots[i].GridX, _multiRobots[i].GridY);
                _robotLastOccupiedGrid[i] = startGrid;

                // 初始出生点作为机器人当前占有的实际受控区域
                _lockedZones[startGrid] = i;
                _robotClaimedZones[i].Add(startGrid);

                List<Point> path = planResult.AgentPaths[i];
                if (path.Count > maxPathSteps) maxPathSteps = path.Count;
                totalSteps += path.Count;
            }

            // 必须在导航状态激活之后再装载路径，否则机器人不会进入等待许可状态
            _isMultiNavigating = true;
            _totalMultiWaitTime = 0.0;
            _finalMultiNavDuration = 0.0;
            _deadlockManager.Reset();
            _lastDeadlockCheckTime = 0.0;

            for (int i = 0; i < _multiRobots.Count; i++)
            {
                _multiRobots[i].AssignPlannedPath(planResult.AgentPaths[i], _multiTargets[i], CellSizeMeters, GridColumns, GridRows);
            }
            SetMultiNavControlsLocked(true);

            // 依据最长步数动态配置物理执行超时上限
            float stepTime = CellSizeMeters / Math.Max(0.5f, (float)_numSpeed.Value);
            _multiNavTimeoutSeconds = Math.Max(15f, maxPathSteps * stepTime * 2.5f + 8f);
            _multiNavStopwatch.Restart();
            _lastMultiProgressTime = _stopwatch.Elapsed.TotalSeconds;
            _lastRemainingTotalSteps = totalSteps;

            _lblMultiInfo.Text = $"调度中: 采用 {selectedAlgorithm} 协同行进 ({planResult.ElapsedMilliseconds}ms)";
            _lblMultiInfo.ForeColor = Color.FromArgb(0, 110, 220);
            _skiaCanvas?.Invalidate();
        }

        private void HandleStopMultiNavigation()
        {
            // 响应用户请求停止多机调度并平滑制动所有机器人，释放全部受控区域资源
            for (int i = 0; i < _multiRobots.Count; i++)
            {
                _multiRobots[i].StopAtNextGrid(CellSizeMeters);
            }

            _lockedZones.Clear();
            _robotClaimedZones.Clear();
            _lockedEdges.Clear();
            _robotClaimedEdges.Clear();
            _agentTimedSchedules.Clear();

            _finalMultiNavDuration = _multiNavStopwatch.Elapsed.TotalSeconds;
            _isMultiNavigating = false;
            _multiNavStopwatch.Stop();
            SetMultiNavControlsLocked(false);

            _lblMultiInfo.Text = "多机调度任务已手动停止 (安全栅格刹停)";
            _lblMultiInfo.ForeColor = Color.FromArgb(200, 80, 40);
            _skiaCanvas?.Invalidate();
        }

        /// <summary>
        /// 当区域内出现多机死锁或复杂停滞时，执行动态全局协同重规划。
        /// 严格将重规划输出的时空预约时间轴与实际物理运行时间（秒表时间）毫秒级对齐，
        /// 确保所有机器人的预约时空窗口与现场真实运行进度绝对匹配，从根源杜绝重规划后的撞车与错位。
        /// </summary>
        private bool TryExecuteGlobalDeadlockReplanning(string triggerReason)
        {
            if (!_isMultiNavigating || _multiRobots == null || _multiRobots.Count == 0)
            {
                return false;
            }

            int n = _multiRobots.Count;

            // 1. 先让所有正在移动的机器人平稳就位在当前栅格中心，杜绝未完成滑行导致的坐标漂移
            for (int i = 0; i < n; i++)
            {
                if (_multiRobots[i].ActionState == RobotActionState.Moving)
                {
                    _multiRobots[i].StopAtNextGrid(CellSizeMeters);
                }
            }

            // 2. 构造以各车当前稳定栅格为动态起点的多机协同任务清单
            List<AgentTask> tasks = new List<AgentTask>(n);
            for (int i = 0; i < n; i++)
            {
                tasks.Add(new AgentTask(i, new Point(_multiRobots[i].GridX, _multiRobots[i].GridY), _multiTargets[i]));
            }

            // 获取当前选定的算法参数
            MultiAgentAlgorithm algo = (MultiAgentAlgorithm)_cmbMultiAlgorithm.SelectedIndex;
            int whcaK = (_numWhcaK != null) ? (int)_numWhcaK.Value : 18;
            double ecbsW = (_numEcbsW != null) ? (double)_numEcbsW.Value : 1.30;

            // 3. 快速多机协同求解 (优先当前算法)
            MultiAgentPlanResult planResult = MultiAgentPathPlanning.Plan(
                tasks,
                GridColumns,
                GridRows,
                algo,
                timeoutMs: 2500,
                whcaWindowK: whcaK,
                ecbsSuboptimality: ecbsW);

            // 若原算法耗时过长或未解出，自动使用次优容错性极高且求解迅速的 ECBS 协同救场
            if (!planResult.Success || planResult.IsTimeout)
            {
                planResult = MultiAgentPathPlanning.Plan(
                    tasks,
                    GridColumns,
                    GridRows,
                    MultiAgentAlgorithm.ECBS,
                    timeoutMs: 2000,
                    whcaWindowK: whcaK,
                    ecbsSuboptimality: 1.50);
            }

            if (!planResult.Success || planResult.IsTimeout)
            {
                return false; // 确实发生全图绝对物理刚性死锁
            }

            // 4. 获取当前真实物理运行时间基准 (秒)
            double currentRealTime = _multiNavStopwatch.Elapsed.TotalSeconds;

            // 5. 原子级重构受控锁与时空预约表，严格将重规划时间轴与实际时间轴精准对齐
            _lockedZones.Clear();
            _robotClaimedZones.Clear();
            _lockedEdges.Clear();
            _robotClaimedEdges.Clear();
            _agentTimedSchedules.Clear();

            if (planResult.TimedSchedules != null)
            {
                for (int i = 0; i < planResult.TimedSchedules.Count; i++)
                {
                    var sched = planResult.TimedSchedules[i];
                    var realAlignedSched = new List<TimedPathPoint>(sched.Count);
                    for (int p = 0; p < sched.Count; p++)
                    {
                        var tp = sched[p];
                        // 核心：将从 0 开始的重规划相对时间序列，加上当前真实时间 currentRealTime，与物理世界绝对时间严格匹配！
                        realAlignedSched.Add(new TimedPathPoint(
                            tp.Grid,
                            tp.PlannedEntryTime + currentRealTime,
                            tp.PlannedExitTime + currentRealTime));
                    }
                    _agentTimedSchedules.Add(realAlignedSched);
                }
            }

            int maxSteps = 0;
            int totalSteps = 0;
            for (int i = 0; i < n; i++)
            {
                _robotClaimedZones[i] = new HashSet<Point>();
                _robotClaimedEdges[i] = new HashSet<long>();
                _agentTimeOffsets[i] = 0.0; // 时间基准已统一对齐至当前实际时钟，重置各车相对偏移
                Point startGrid = new Point(_multiRobots[i].GridX, _multiRobots[i].GridY);
                _robotLastOccupiedGrid[i] = startGrid;

                _lockedZones[startGrid] = i;
                _robotClaimedZones[i].Add(startGrid);

                List<Point> path = planResult.AgentPaths[i];
                if (path.Count > maxSteps) maxSteps = path.Count;
                totalSteps += path.Count;
            }

            // 6. 无缝下发新路径并激活等待许可
            for (int i = 0; i < n; i++)
            {
                _multiRobots[i].AssignPlannedPath(planResult.AgentPaths[i], _multiTargets[i], CellSizeMeters, GridColumns, GridRows);
            }

            // 7. 刷新推进时间戳与死锁管理器状态
            _deadlockManager.Reset();
            _lastDeadlockCheckTime = _stopwatch.Elapsed.TotalSeconds;
            _lastMultiProgressTime = _stopwatch.Elapsed.TotalSeconds;
            _lastRemainingTotalSteps = totalSteps;

            _lblMultiInfo.Text = $"死锁自动消解: 全局时空预约已与实际时间精准匹配，机群重新解冻！({planResult.ElapsedMilliseconds}ms)";
            _lblMultiInfo.ForeColor = Color.FromArgb(0, 160, 80);
            _skiaCanvas?.Invalidate();
            return true;
        }

        private void SetMultiNavControlsLocked(bool locked)
        {
            // 在多机任务开始到结束之前严格锁定算法选择与任务配置，防止非法更换状态
            if (_cmbMultiAlgorithm != null) _cmbMultiAlgorithm.Enabled = !locked;
            if (_numWhcaK != null) _numWhcaK.Enabled = !locked;
            if (_numEcbsW != null) _numEcbsW.Enabled = !locked;
            if (_cmbRobotCount != null) _cmbRobotCount.Enabled = !locked;
            if (_btnRandomizeRobots != null) _btnRandomizeRobots.Enabled = !locked;
            if (_btnStartMultiNav != null) _btnStartMultiNav.Enabled = !locked;
            if (_btnStopMultiNav != null) _btnStopMultiNav.Enabled = locked;
            if (_cmbMode != null) _cmbMode.Enabled = !locked;
        }

        #region 动态受控区域锁定与资源释放闭环调度体系

        /// <summary>
        /// 流程 2~4：接近下一个受控区域 -> 检查时间预约有效与实际区域安全 -> 获得进入许可
        /// </summary>
        private int AcquireSegmentPermission(int robotIndex, List<Point> segmentZones, float cellSizeMeters)
        {
            // 无待行进栅格（路径已尽）：无需预约
            if (segmentZones == null || segmentZones.Count == 0)
            {
                return 0;
            }

            Robot self = _multiRobots[robotIndex];

            // 原地转向段：唯一元素即当前所在格，仅需确认本格仍归自己所有
            if (segmentZones.Count == 1 && segmentZones[0].X == self.GridX && segmentZones[0].Y == self.GridY)
            {
                return 0;
            }

            // 实际区域安全阈值取单车半外形 (0.20m)：仅拦截"他车车身确实压在该格内"。
            // 相邻格中心间距 0.55m 远大于两车半外形之和 0.40m，邻格停车天然安全；
            // 阈值若放大会把邻格静止车误判为障碍，导致机群互相冻结。
            float bodyHalf = (Robot.ChassisLengthMeters + Robot.ChassisWidthMeters) * 0.25f;
            float clearanceSq = bodyHalf * bodyHalf;

            // 沿路径向前贪心预约最长可用前缀：遇到第一个被占栅格或被锁跨越边即停止
            int granted = 0;
            Point prevZone = new Point(self.GridX, self.GridY);

            for (int k = 0; k < segmentZones.Count; k++)
            {
                Point zone = segmentZones[k];

                // 1. 目标栅格顶点锁检验
                if (_lockedZones.TryGetValue(zone, out int occupantId) && occupantId != robotIndex)
                {
                    break;
                }

                // 2. 动态无向边锁检验 (绝对禁止相向对穿边冲突 Edge Conflict)
                if (zone != prevZone)
                {
                    long edgeKey = GetUndirectedEdgeKey(prevZone, zone);
                    if (_lockedEdges.TryGetValue(edgeKey, out int edgeOwner) && edgeOwner != robotIndex)
                    {
                        break; // 该边正在被其他机器人通行，严禁逆向或重叠进入！
                    }
                }

                float cx = (zone.X + 0.5f) * cellSizeMeters;
                float cy = (zone.Y + 0.5f) * cellSizeMeters;
                bool blocked = false;
                for (int j = 0; j < _multiRobots.Count; j++)
                {
                    if (j == robotIndex) continue;

                    Robot other = _multiRobots[j];
                    float dx = other.PositionX - cx;
                    float dy = other.PositionY - cy;
                    if (dx * dx + dy * dy < clearanceSq)
                    {
                        blocked = true;
                        break;
                    }
                }
                if (blocked)
                {
                    break;
                }

                // 登记顶点锁定
                _lockedZones[zone] = robotIndex;
                if (_robotClaimedZones.TryGetValue(robotIndex, out var claimed))
                {
                    claimed.Add(zone);
                }

                // 登记无向边锁定 (独占当前跨越边)
                if (zone != prevZone)
                {
                    long edgeKey = GetUndirectedEdgeKey(prevZone, zone);
                    _lockedEdges[edgeKey] = robotIndex;
                    if (_robotClaimedEdges.TryGetValue(robotIndex, out var cEdges))
                    {
                        cEdges.Add(edgeKey);
                    }
                }

                prevZone = zone;
                granted++;
            }

            return granted;
        }

        /// <summary>
        /// 流程 5~7：机器人通过受控区域 -> 动态释放已安全脱离的历史受控区域。
        /// 保持长直段整段预约，但在释放端将保护区缩短为：根据最大速度与加速度计算的最大制动距离加上一个栅格的安全距离。
        /// 只要车身后方脱离间距达到此安全保护单位，即时动态解离该历史栅格，实现高吞吐流水线跟驰，同时当前所在格始终锁定。
        /// </summary>
        private void CheckAndReleasePastZones(int robotIndex, float cellSizeMeters)
        {
            Robot robot = _multiRobots[robotIndex];
            if (!_robotClaimedZones.TryGetValue(robotIndex, out var claimed))
            {
                return;
            }

            // 当前车身实际所在栅格必须始终持有占用锁，防止他车获得该格通行许可
            Point occupiedGrid = new Point(robot.GridX, robot.GridY);
            if (!_lockedZones.TryGetValue(occupiedGrid, out int curOwner) || curOwner != robotIndex)
            {
                _lockedZones[occupiedGrid] = robotIndex;
                claimed.Add(occupiedGrid);
            }

            // 核心安全优化：判定机器人是否已完成路径行进并安全停靠在自身专属目标点
            Point target = (robotIndex < _multiTargets.Count) ? _multiTargets[robotIndex] : Point.Empty;
            bool isAtGoal = (robot.GridX == target.X && robot.GridY == target.Y &&
                             robot.NavState == RobotNavState.Idle &&
                             robot.ActionState == RobotActionState.Idle);

            if (isAtGoal)
            {
                // 到达目标点后：仅保留自身目标栅格锁定，无条件释放认领集合中所有其他历史栅格（彻底解离前驱入口格等）
                List<Point> goalReleased = null;
                foreach (Point zone in claimed)
                {
                    if (zone != occupiedGrid)
                    {
                        if (_lockedZones.TryGetValue(zone, out int occupant) && occupant == robotIndex)
                        {
                            _lockedZones.Remove(zone);
                        }
                        if (goalReleased == null) goalReleased = new List<Point>();
                        goalReleased.Add(zone);
                    }
                }

                if (goalReleased != null)
                {
                    for (int k = 0; k < goalReleased.Count; k++)
                    {
                        claimed.Remove(goalReleased[k]);
                    }
                }

                // 解离全部认领的边锁 (已停泊就位，不再占用任何跨越边)
                if (_robotClaimedEdges.TryGetValue(robotIndex, out var cEdges))
                {
                    foreach (long ek in cEdges)
                    {
                        if (_lockedEdges.TryGetValue(ek, out int eo) && eo == robotIndex)
                        {
                            _lockedEdges.Remove(ek);
                        }
                    }
                    cEdges.Clear();
                }

                _robotLastOccupiedGrid[robotIndex] = occupiedGrid;
                return;
            }

            // 行进中状态与当前合批直线段端点及方向
            bool moving = robot.ActionState == RobotActionState.Moving;
            Point segStart = robot.SegmentStartGrid;
            Point segEnd = robot.SegmentEndGrid;
            int dirX = Math.Sign(segEnd.X - segStart.X);
            int dirY = Math.Sign(segEnd.Y - segStart.Y);

            // 基础安全脱离间距：车身半外形 (约 0.20m)
            float bodyHalf = (Robot.ChassisLengthMeters + Robot.ChassisWidthMeters) * 0.25f;

            // 依据最大速度与加速度计算最大制动距离: S_brake = v^2 / (2 * a)
            float maxSpeed = Math.Max(0.1f, robot.MaxSpeed);
            float accel = Math.Max(0.1f, robot.Acceleration);
            float maxBrakeDistance = (maxSpeed * maxSpeed) / (2f * accel);

            // 动态安全保护距离单位：最大制动距离 + 一个栅格的安全距离 + 车身半外形
            // 确保后方跟随车辆获准进入该释放栅格时，与本车车尾之间始终保持大于物理制动极限的安全间距，从力学上杜绝追尾
            float segmentProtectDistance = maxBrakeDistance + cellSizeMeters + bodyHalf;

            // 静止或非当前合批航道常规驶离安全位移平方 (1 格 + 车身半长)
            float baseReleaseClearance = cellSizeMeters + bodyHalf;
            float baseReleaseClearanceSq = baseReleaseClearance * baseReleaseClearance;

            List<Point> released = null;
            foreach (Point zone in claimed)
            {
                // 1. 当前所在网格永不释放
                if (zone == occupiedGrid)
                {
                    continue;
                }

                float zoneCenterX = (zone.X + 0.5f) * cellSizeMeters;
                float zoneCenterY = (zone.Y + 0.5f) * cellSizeMeters;
                float dx = robot.PositionX - zoneCenterX;
                float dy = robot.PositionY - zoneCenterY;

                // 2. 判定该栅格是否属于当前正在行进的合批直线段航道
                bool onCurrentSegment = moving && (dirX != 0 || dirY != 0) &&
                                        IsOnSegmentLine(zone, segStart, segEnd, dirX, dirY);

                if (onCurrentSegment)
                {
                    // 计算沿移动方向车身超出该栅格中心的物理前向位移
                    float forwardDist = dx * dirX + dy * dirY;

                    // 若该栅格在车身前方 (forwardDist <= 0)，或车身后方安全间距尚未达到“最大制动距离 + 1格安全距离”，必须持续锁定保护
                    if (forwardDist < segmentProtectDistance)
                    {
                        continue;
                    }
                }
                else
                {
                    // 对于历史转弯点或非当前段直线栅格，确认欧氏距离彻底驶离
                    if (dx * dx + dy * dy < baseReleaseClearanceSq)
                    {
                        continue;
                    }
                }

                // 满足安全释放条件：从全局受控区域锁中解离该栅格资源
                if (_lockedZones.TryGetValue(zone, out int occupant) && occupant == robotIndex)
                {
                    _lockedZones.Remove(zone);
                }
                if (released == null) released = new List<Point>();
                released.Add(zone);
            }

            if (released != null)
            {
                for (int k = 0; k < released.Count; k++)
                {
                    claimed.Remove(released[k]);
                }
            }

            // 动态解离机器人已彻底驶离的历史边锁
            if (_robotClaimedEdges.TryGetValue(robotIndex, out var activeEdges) && activeEdges.Count > 0)
            {
                List<long> edgesToRelease = null;
                foreach (long ek in activeEdges)
                {
                    int uX = (int)(ek >> 48);
                    int uY = (int)((ek >> 32) & 0xFFFF);
                    int vX = (int)((ek >> 16) & 0xFFFF);
                    int vY = (int)(ek & 0xFFFF);
                    Point pU = new Point(uX, uY);
                    Point pV = new Point(vX, vY);

                    if (!claimed.Contains(pU) || !claimed.Contains(pV))
                    {
                        if (_lockedEdges.TryGetValue(ek, out int eo) && eo == robotIndex)
                        {
                            _lockedEdges.Remove(ek);
                        }
                        if (edgesToRelease == null) edgesToRelease = new List<long>();
                        edgesToRelease.Add(ek);
                    }
                }

                if (edgesToRelease != null)
                {
                    for (int k = 0; k < edgesToRelease.Count; k++)
                    {
                        activeEdges.Remove(edgesToRelease[k]);
                    }
                }
            }

            _robotLastOccupiedGrid[robotIndex] = occupiedGrid;
        }

        /// <summary>
        /// 判断某栅格是否落在机器人当前合批直线段的整条航道区间内（从起点直至终点覆盖的所有栅格）。
        /// </summary>
        private static bool IsOnSegmentLine(Point zone, Point segStart, Point segEnd, int dirX, int dirY)
        {
            if (dirX != 0)
            {
                if (zone.Y != segStart.Y) return false;
                int minX = Math.Min(segStart.X, segEnd.X);
                int maxX = Math.Max(segStart.X, segEnd.X);
                return zone.X >= minX && zone.X <= maxX;
            }
            if (dirY != 0)
            {
                if (zone.X != segStart.X) return false;
                int minY = Math.Min(segStart.Y, segEnd.Y);
                int maxY = Math.Max(segStart.Y, segEnd.Y);
                return zone.Y >= minY && zone.Y <= maxY;
            }
            return false;
        }

        /// <summary>
        /// 偏差动态调整与时钟顺延：当机器人在当前格等待让行时，将该机器人的后续计划时间预约真实向后动态顺延，
        /// 确保预约表的时间窗口始终与现场实际物理小车时钟保持毫秒级动态对齐，彻底消除时间漂移。
        /// </summary>
        private void AdjustTimeOffsetOnStall(int robotIndex, double deltaSeconds)
        {
            if (robotIndex >= 0 && robotIndex < _agentTimeOffsets.Length)
            {
                _agentTimeOffsets[robotIndex] += deltaSeconds;
            }

            // 真正将物理等待时间顺延至机器人尚未通过的所有时空预约航点中
            if (robotIndex >= 0 && robotIndex < _agentTimedSchedules.Count && robotIndex < _multiRobots.Count)
            {
                var schedule = _agentTimedSchedules[robotIndex];
                if (schedule != null && schedule.Count > 0)
                {
                    Robot robot = _multiRobots[robotIndex];
                    Point curGrid = new Point(robot.GridX, robot.GridY);

                    // 定位当前机器人所在的航点起点索引
                    int startIdx = 0;
                    for (int k = 0; k < schedule.Count; k++)
                    {
                        if (schedule[k].Grid == curGrid)
                        {
                            startIdx = k;
                            break;
                        }
                    }

                    // 将后续所有尚未通行的时空预约区间统一向后顺延 deltaSeconds
                    for (int k = startIdx; k < schedule.Count; k++)
                    {
                        var tp = schedule[k];
                        tp.PlannedEntryTime += deltaSeconds;
                        tp.PlannedExitTime += deltaSeconds;
                        schedule[k] = tp;
                    }
                }
            }
        }

        /// <summary>
        /// 独立安全系统物理兜底 (作为底层最后防线，无条件确保车身外形不发生实体碰撞)
        /// </summary>
        private void IndependentSafetySystemCheck(float cellSizeMeters)
        {
            float safetyLimitDist = Robot.ChassisWidthMeters * 0.90f; // 0.342m 极限接触阈值
            float limitDistSq = safetyLimitDist * safetyLimitDist;
            float releaseDistSq = safetyLimitDist * 1.30f * safetyLimitDist * 1.30f;

            for (int i = 0; i < _multiRobots.Count; i++)
            {
                for (int j = i + 1; j < _multiRobots.Count; j++)
                {
                    float dx = _multiRobots[i].PositionX - _multiRobots[j].PositionX;
                    float dy = _multiRobots[i].PositionY - _multiRobots[j].PositionY;
                    float dSq = dx * dx + dy * dy;
                    long pairKey = ((long)i << 32) | (uint)j;

                    if (dSq < limitDistSq)
                    {
                        if (!_activeCollisionPairs.Contains(pairKey))
                        {
                            _activeCollisionPairs.Add(pairKey);
                            _collisionCount++;
                        }
                    }
                    else if (dSq > releaseDistSq)
                    {
                        _activeCollisionPairs.Remove(pairKey);
                    }
                }
            }
        }

        #endregion

        private void HandleRandomizeRobots()
        {
            // 响应重新随机分布多机器人按钮请求
            if (_cmbRobotCount != null && _cmbRobotCount.SelectedItem is int count)
            {
                GenerateRandomRobots(count);
            }
        }

        private void OnRobotCountSelectedIndexChanged(object sender, EventArgs e)
        {
            // 响应机器人数量选择变更并在地图上随机生成相应数量的机器人
            if (_cmbRobotCount != null && _cmbRobotCount.SelectedItem is int count)
            {
                GenerateRandomRobots(count);
            }
        }

        private void GenerateRandomRobots(int count)
        {
            // 在 30x30 栅格地图中随机生成指定数量且出生点互不重叠的机器人，并为每台机器人分配对角半区专属目标点。
            // count: 用户选定的机器人数量 (5, 10, 20, 50, 100, 200)
            _multiRobots.Clear();
            _multiTargets.Clear();
            _collisionCount = 0;
            _multiPlanElapsedMs = -1;
            _activeCollisionPairs.Clear();

            int totalCells = GridColumns * GridRows;
            if (count > totalCells) count = totalCells;

            // 避开地图最正中心4个无法满足对角距离严格大于15格的交界栅格 (14,14),(14,15),(15,14),(15,15)
            HashSet<int> centerCells = new HashSet<int>
            {
                14 * GridColumns + 14,
                14 * GridColumns + 15,
                15 * GridColumns + 14,
                15 * GridColumns + 15
            };

            // 使用集合排除法保证机器人的初始出生位置互不重叠
            HashSet<int> usedRobotPositions = new HashSet<int>();
            while (usedRobotPositions.Count < count)
            {
                int cand = _random.Next(totalCells);
                if (!centerCells.Contains(cand))
                {
                    usedRobotPositions.Add(cand);
                }
            }

            float currentSpeed = (_numSpeed != null) ? (float)_numSpeed.Value : 1.50f;
            float currentAccel = (_numAccel != null) ? (float)_numAccel.Value : 2.00f;

            foreach (int pos in usedRobotPositions)
            {
                int gx = pos % GridColumns;
                int gy = pos / GridColumns;
                RobotOrientation ori = (RobotOrientation)_random.Next(4);

                Robot robot = new Robot(gx, gy, CellSizeMeters, ori)
                {
                    MaxSpeed = currentSpeed,
                    Acceleration = currentAccel,
                    ExternalScheduling = true
                };
                _multiRobots.Add(robot);
            }

            // 根据机器人数量一次性设置单独属于每个机器人的对角半区目标点
            GenerateDiagonalTargets();

            if (_lblMultiInfo != null)
            {
                _lblMultiInfo.Text = $"机群: {_multiRobots.Count}台 | 目标: {_multiTargets.Count}个 | 碰撞: 0次 (就绪待命)";
                _lblMultiInfo.ForeColor = Color.FromArgb(40, 140, 60);
            }

            _skiaCanvas?.Invalidate();
        }

        private void GenerateDiagonalTargets()
        {
            // 为机群中的每台机器人设置一次性专属目标点，严格位于其对角半区且水平或垂直距离至少一个大于15栅格。
            _multiTargets.Clear();
            HashSet<Point> usedTargetPoints = new HashSet<Point>();

            for (int i = 0; i < _multiRobots.Count; i++)
            {
                Robot robot = _multiRobots[i];
                Point target = FindDiagonalTargetForRobot(robot, usedTargetPoints);
                usedTargetPoints.Add(target);
                _multiTargets.Add(target);
            }
        }

        private Point FindDiagonalTargetForRobot(Robot robot, HashSet<Point> usedTargetPoints)
        {
            // 为单个机器人寻找属于其对角半区且距离约束满足(|dx|>15 或 |dy|>15)的唯一未占用目标栅格。
            // 30x30 地图以中线 15 为界划分左(0~14)/右(15~29)与上(0~14)/下(15~29)四个象限半区。
            int robotQuadX = (robot.GridX < 15) ? 0 : 1;
            int robotQuadY = (robot.GridY < 15) ? 0 : 1;

            // 对角象限索引变换
            int diagQuadX = 1 - robotQuadX;
            int diagQuadY = 1 - robotQuadY;

            int minX = (diagQuadX == 0) ? 0 : 15;
            int maxX = (diagQuadX == 0) ? 14 : 29;
            int minY = (diagQuadY == 0) ? 0 : 15;
            int maxY = (diagQuadY == 0) ? 14 : 29;

            List<Point> candidates = new List<Point>();
            List<Point> fallbackCandidates = new List<Point>();

            for (int x = minX; x <= maxX; x++)
            {
                for (int y = minY; y <= maxY; y++)
                {
                    Point pt = new Point(x, y);
                    if (usedTargetPoints.Contains(pt))
                    {
                        continue;
                    }

                    int dx = Math.Abs(x - robot.GridX);
                    int dy = Math.Abs(y - robot.GridY);

                    fallbackCandidates.Add(pt);

                    // 水平距离或者垂直距离中至少有一个大于 15 个栅格
                    if (dx > 15 || dy > 15)
                    {
                        candidates.Add(pt);
                    }
                }
            }

            // 优先从满足对角半区且距离 >15 约束的候选栅格中随机抽取
            if (candidates.Count > 0)
            {
                int pickIndex = _random.Next(candidates.Count);
                return candidates[pickIndex];
            }

            // 极端备用兜底：从对角半区未被占用的格子中选取距离起点最远的栅格
            if (fallbackCandidates.Count > 0)
            {
                fallbackCandidates.Sort((a, b) =>
                {
                    int distA = Math.Max(Math.Abs(a.X - robot.GridX), Math.Abs(a.Y - robot.GridY));
                    int distB = Math.Max(Math.Abs(b.X - robot.GridX), Math.Abs(b.Y - robot.GridY));
                    return distB.CompareTo(distA);
                });
                return fallbackCandidates[0];
            }

            // 极端兜底点
            return new Point((minX + maxX) / 2, (minY + maxY) / 2);
        }

        private void DrawMultiRobotTargets(SKCanvas canvas, float cellPixels)
        {
            // 在地图视口中批量渲染多机调度模式下每台机器人的专属目标点、对角连线与编号角标。
            if (_multiTargets == null || _multiTargets.Count == 0)
            {
                return;
            }

            bool showGuideLine = (_chkShowTargetLines != null && _chkShowTargetLines.Checked);

            float pixelsPerMeter = (BaseCellPixels * _scale) / CellSizeMeters;
            float badgeRadius = Math.Max(5f, Math.Min(9f, 6.5f * _scale));
            float textSize = Math.Max(7.5f, Math.Min(10.5f, 8f * _scale));

            using (var targetBgPaint = new SKPaint
            {
                Color = new SKColor(230, 50, 50, 45),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            using (var borderPaint = new SKPaint
            {
                Color = new SKColor(220, 40, 40, 200),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(1.2f, 1.8f * _scale),
                IsAntialias = true
            })
            using (var ringPaint = new SKPaint
            {
                Color = new SKColor(220, 30, 30, 220),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(1f, 1.6f * _scale),
                IsAntialias = true
            })
            using (var centerDotPaint = new SKPaint
            {
                Color = new SKColor(210, 25, 25, 230),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            using (var guideLinePaint = new SKPaint
            {
                Color = new SKColor(140, 80, 220, 85),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(1f, 1.5f * _scale),
                IsAntialias = true,
                PathEffect = SKPathEffect.CreateDash(new float[] { 4f * _scale, 4f * _scale }, 0f)
            })
            using (var badgeBgPaint = new SKPaint
            {
                Color = new SKColor(210, 35, 35, 230),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            using (var badgeBorderPaint = new SKPaint
            {
                Color = new SKColor(255, 255, 255, 220),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1f,
                IsAntialias = true
            })
            using (var badgeTextPaint = new SKPaint
            {
                Color = SKColors.White,
                TextSize = textSize,
                IsAntialias = true,
                Typeface = YaHeiBoldTypeface,
                TextAlign = SKTextAlign.Center
            })
            {
                float outerRadius = cellPixels * 0.35f;
                float innerRadius = cellPixels * 0.18f;
                float centerDotRadius = Math.Max(1.2f, cellPixels * 0.08f);

                for (int i = 0; i < _multiTargets.Count; i++)
                {
                    Point target = _multiTargets[i];
                    float cellX = _offsetX + target.X * cellPixels;
                    float cellY = _offsetY + target.Y * cellPixels;
                    float centerX = cellX + cellPixels * 0.5f;
                    float centerY = cellY + cellPixels * 0.5f;

                    // 1. 若开启连线勾选，绘制从机器人实时物理位置到专属对角目标点的导引虚线 (支持跟随运动)
                    if (showGuideLine && i < _multiRobots.Count)
                    {
                        Robot r = _multiRobots[i];
                        float robotCenterX = _offsetX + r.PositionX * pixelsPerMeter;
                        float robotCenterY = _offsetY + r.PositionY * pixelsPerMeter;
                        canvas.DrawLine(robotCenterX, robotCenterY, centerX, centerY, guideLinePaint);
                    }

                    // 2. 绘制专属目标栅格底衬与边框
                    canvas.DrawRect(cellX, cellY, cellPixels, cellPixels, targetBgPaint);
                    canvas.DrawRect(cellX, cellY, cellPixels, cellPixels, borderPaint);

                    // 3. 绘制靶心同心圆与中心点
                    canvas.DrawCircle(centerX, centerY, outerRadius, ringPaint);
                    canvas.DrawCircle(centerX, centerY, innerRadius, ringPaint);
                    canvas.DrawCircle(centerX, centerY, centerDotRadius, centerDotPaint);

                    // 4. 目标点右上角标注小字体专属编号角标 (固定常驻显示)
                    float bX = cellX + cellPixels * 0.82f;
                    float bY = cellY + cellPixels * 0.18f;

                    canvas.DrawCircle(bX, bY, badgeRadius, badgeBgPaint);
                    canvas.DrawCircle(bX, bY, badgeRadius, badgeBorderPaint);

                    string numText = (i + 1).ToString();
                    float textBaseline = bY + textSize * 0.36f;
                    canvas.DrawText(numText, bX, textBaseline, badgeTextPaint);
                }
            }
        }

        private void DrawMultiRobotBadges(SKCanvas canvas, float cellPixels)
        {
            // 在多机器人实体的右上角标注固定常驻编号徽章，随机器人平滑位移运动实时跟随。
            if (_multiRobots == null || _multiRobots.Count == 0)
            {
                return;
            }

            float pixelsPerMeter = (BaseCellPixels * _scale) / CellSizeMeters;
            float robotPixelWidth = Robot.ChassisWidthMeters * pixelsPerMeter;

            float badgeRadius = Math.Max(5f, Math.Min(9f, 6.5f * _scale));
            float textSize = Math.Max(7.5f, Math.Min(10.5f, 8f * _scale));

            using (var badgeBgPaint = new SKPaint
            {
                Color = new SKColor(25, 45, 75, 230),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            using (var badgeBorderPaint = new SKPaint
            {
                Color = new SKColor(255, 255, 255, 220),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1f,
                IsAntialias = true
            })
            using (var textPaint = new SKPaint
            {
                Color = SKColors.White,
                TextSize = textSize,
                IsAntialias = true,
                Typeface = YaHeiBoldTypeface,
                TextAlign = SKTextAlign.Center
            })
            {
                for (int i = 0; i < _multiRobots.Count; i++)
                {
                    Robot r = _multiRobots[i];
                    // 使用实时连续物理坐标，保证机器人在加减速移动和航向运动过程中角标始终严密跟随
                    float screenX = _offsetX + r.PositionX * pixelsPerMeter;
                    float screenY = _offsetY + r.PositionY * pixelsPerMeter;

                    // 固定在机器人屏幕坐标系的右上角方向，文字水平正立，不随底盘旋转
                    float badgeX = screenX + robotPixelWidth * 0.42f;
                    float badgeY = screenY - robotPixelWidth * 0.42f;

                    canvas.DrawCircle(badgeX, badgeY, badgeRadius, badgeBgPaint);
                    canvas.DrawCircle(badgeX, badgeY, badgeRadius, badgeBorderPaint);

                    string numText = (i + 1).ToString();
                    float textBaseline = badgeY + textSize * 0.36f;
                    canvas.DrawText(numText, badgeX, textBaseline, textPaint);
                }
            }
        }

        private void DrawMultiPlannedPaths(SKCanvas canvas, float cellPixels)
        {
            // 在多机调度模式下批量绘制每台机器人的正交无冲突规划轨迹折线。
            if (_multiRobots == null || _multiRobots.Count == 0)
            {
                return;
            }

            using (var pathLine = new SKPath())
            using (var linePaint = new SKPaint
            {
                Color = new SKColor(0, 160, 240, 130),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(1.6f, 2.4f * _scale),
                StrokeCap = SKStrokeCap.Round,
                StrokeJoin = SKStrokeJoin.Round,
                IsAntialias = true
            })
            {
                for (int r = 0; r < _multiRobots.Count; r++)
                {
                    List<Point> pList = _multiRobots[r].CurrentPath;
                    if (pList == null || pList.Count < 2)
                    {
                        continue;
                    }

                    pathLine.Reset();
                    float firstPx = _offsetX + (pList[0].X + 0.5f) * cellPixels;
                    float firstPy = _offsetY + (pList[0].Y + 0.5f) * cellPixels;
                    pathLine.MoveTo(firstPx, firstPy);

                    for (int i = 1; i < pList.Count; i++)
                    {
                        float px = _offsetX + (pList[i].X + 0.5f) * cellPixels;
                        float py = _offsetY + (pList[i].Y + 0.5f) * cellPixels;
                        pathLine.LineTo(px, py);
                    }

                    canvas.DrawPath(pathLine, linePaint);
                }
            }
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
            for (int i = 0; i < _multiRobots.Count; i++)
            {
                _multiRobots[i].MaxSpeed = newSpeed;
            }
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
            for (int i = 0; i < _multiRobots.Count; i++)
            {
                _multiRobots[i].Acceleration = newAccel;
            }
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
            if (_cmbMode != null) _cmbMode.Enabled = isIdle;
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

        private void OnModeSelectedIndexChanged(object sender, EventArgs e)
        {
            // 响应仿真运行模式切换并在空闲状态下生效
            if (_robot.NavState != RobotNavState.Idle || _isMultiNavigating)
            {
                MessageBox.Show(this, "机器人集群正在运动中，请在任务完成或停止后再切换运行模式！", "模式切换提示", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                _cmbMode.SelectedIndex = (_currentMode == SimulationMode.SingleRobot) ? 0 : 1;
                return;
            }

            SimulationMode selectedMode = (_cmbMode.SelectedIndex == 1)
                ? SimulationMode.MultiRobot
                : SimulationMode.SingleRobot;

            if (_currentMode == selectedMode)
            {
                return;
            }

            _currentMode = selectedMode;
            OnSimulationModeChanged(_currentMode);
        }

        private void OnSimulationModeChanged(SimulationMode mode)
        {
            // 处理仿真运行模式切换后的界面状态更新与提示
            UpdateModeControlsLayout();

            if (mode == SimulationMode.SingleRobot)
            {
                if (_selectedTarget.HasValue)
                {
                    float physX = (_selectedTarget.Value.X + 0.5f) * CellSizeMeters;
                    float physY = (_selectedTarget.Value.Y + 0.5f) * CellSizeMeters;
                    _lblTargetInfo.Text = "目标网格: [" + _selectedTarget.Value.X + ", " + _selectedTarget.Value.Y + "] (" + physX.ToString("0.00") + "m, " + physY.ToString("0.00") + "m)";
                    _lblTargetInfo.ForeColor = Color.FromArgb(0, 110, 220);
                }
                else
                {
                    _lblTargetInfo.Text = "单机模式: 未设置目标 (在地图中单击网格)";
                    _lblTargetInfo.ForeColor = Color.FromArgb(120, 130, 140);
                }
            }
            else
            {
                if (_multiRobots.Count == 0)
                {
                    int defaultCount = 20;
                    if (_cmbRobotCount != null && _cmbRobotCount.SelectedItem is int selCount)
                    {
                        defaultCount = selCount;
                    }
                    GenerateRandomRobots(defaultCount);
                }
                else if (_lblMultiInfo != null)
                {
                    _lblMultiInfo.Text = "机群: " + _multiRobots.Count + "台 | 目标: " + _multiTargets.Count + "个 (对角>15格)";
                }
            }

            _skiaCanvas?.Invalidate();
        }

        private void OnAlgorithmSelectedIndexChanged(object sender, EventArgs e)
        {
            // 响应单机寻路算法选择变更并在空闲状态下同步至单机机器人状态机。
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
            // 在单机模式下全局拦截键盘输入以驱动机器人前进、转向及复位
            if (_currentMode == SimulationMode.SingleRobot)
            {
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
            }

            return base.ProcessCmdKey(ref msg, keyData);
        }

        private void OnSimulationTick(object sender, EventArgs e)
        {
            // 驱动物理仿真时间步长，仅在实体运动或姿态变化时按需重绘，并在多机执行中检测死锁与完成。
            double currentSeconds = _stopwatch.Elapsed.TotalSeconds;
            float deltaTime = (float)(currentSeconds - _lastElapsedSeconds);
            _lastElapsedSeconds = currentSeconds;

            if (deltaTime > 0.1f)
            {
                deltaTime = 0.1f;
            }

            bool hasMotion = false;

            if (_currentMode == SimulationMode.SingleRobot)
            {
                if (_robot.ActionState != RobotActionState.Idle)
                {
                    _robot.Update(deltaTime, CellSizeMeters);
                    hasMotion = true;
                }
            }
            else
            {
                int currentTotalRemainingSteps = 0;
                bool allReached = true;

                // 闭环流程 5~7：通过受控区域 -> 实际物理位置确认车身和载荷完全离开 -> 释放实际占用
                for (int i = 0; i < _multiRobots.Count; i++)
                {
                    CheckAndReleasePastZones(i, CellSizeMeters);
                }

                // 闭环流程 2~4：接近受控区域 -> 检查时间预约有效与实际区域安全 -> 获得进入许可 -> 启动动力学前进
                for (int i = 0; i < _multiRobots.Count; i++)
                {
                    Robot r = _multiRobots[i];
                    if (r.ActionState != RobotActionState.Idle)
                    {
                        // 正在受控区域内按平滑梯形/三角形动力学平稳行进，到达该段终点自然停靠
                        r.Update(deltaTime, CellSizeMeters, GridColumns, GridRows);
                        hasMotion = true;
                    }
                    else if (_isMultiNavigating && r.IsAwaitingPermission)
                    {
                        // 停在栅格中心等待许可：预约最长可用前缀后按完整梯形/三角形动力学放行
                        List<Point> segmentZones = r.PeekSegmentZones();
                        bool isTurnInPlace = segmentZones.Count == 1 &&
                                             segmentZones[0].X == r.GridX && segmentZones[0].Y == r.GridY;
                        int granted = AcquireSegmentPermission(i, segmentZones, CellSizeMeters);

                        if (granted > 0 || isTurnInPlace || segmentZones.Count == 0)
                        {
                            r.GrantStepPermission(GridColumns, GridRows, CellSizeMeters, granted);
                            r.Update(deltaTime, CellSizeMeters, GridColumns, GridRows);
                            hasMotion = true;
                        }
                        else
                        {
                            // 前方区域未释放：原地平稳等待，完整保留后续路径并顺延时间预约
                            AdjustTimeOffsetOnStall(i, deltaTime);
                            _totalMultiWaitTime += deltaTime;
                        }
                    }

                    // 检查是否到达专属目标
                    Point target = (i < _multiTargets.Count) ? _multiTargets[i] : Point.Empty;
                    bool isAtGoal = (r.GridX == target.X && r.GridY == target.Y && r.NavState == RobotNavState.Idle);
                    if (!isAtGoal)
                    {
                        allReached = false;
                    }

                    if (r.CurrentPath != null)
                    {
                        currentTotalRemainingSteps += r.CurrentPath.Count;
                    }
                }

                // 独立安全系统物理兜底 (作为底层最后防线，无条件确保车身外形不发生实体碰撞)
                IndependentSafetySystemCheck(CellSizeMeters);

                // 多机协同调度任务生命周期与死锁检测
                if (_isMultiNavigating)
                {
                    // 1. 全部到达目标点判定：任务成功完成
                    if (allReached)
                    {
                        _finalMultiNavDuration = _multiNavStopwatch.Elapsed.TotalSeconds;
                        _isMultiNavigating = false;
                        _multiNavStopwatch.Stop();
                        SetMultiNavControlsLocked(false);
                        _lblMultiInfo.Text = $"任务成功: 全部机群已安全到达专属目标点！(耗时 {_finalMultiNavDuration:0.0}s)";
                        _lblMultiInfo.ForeColor = Color.FromArgb(40, 160, 80);
                    }
                    else
                    {
                        // 2. 推进进度检测
                        if (currentTotalRemainingSteps < _lastRemainingTotalSteps)
                        {
                            _lastRemainingTotalSteps = currentTotalRemainingSteps;
                            _lastMultiProgressTime = currentSeconds;
                        }

                        // 3. 周期性执行标准工业级死锁、堵塞检测与应对机制 (每 0.25 秒执行一次)
                        bool isTimeExceeded = (_multiNavStopwatch.Elapsed.TotalSeconds > _multiNavTimeoutSeconds);
                        bool isConfirmedDeadlock = false;
                        string deadlockDetails = string.Empty;

                        if (currentSeconds - _lastDeadlockCheckTime >= 0.25)
                        {
                            _lastDeadlockCheckTime = currentSeconds;
                            var report = _deadlockManager.DiagnoseAndResolve(
                                _multiRobots,
                                _multiTargets,
                                _lockedZones,
                                _lockedEdges,
                                GridColumns,
                                GridRows,
                                CellSizeMeters,
                                currentSeconds);

                            if (report.Status == DeadlockStatus.NeedGlobalReplanning)
                            {
                                // 检测到死锁环，自动触发具备实际物理时间精准匹配的全局协同重规划打破僵局
                                bool replanned = TryExecuteGlobalDeadlockReplanning(report.ResolutionDetails);
                                if (!replanned)
                                {
                                    isConfirmedDeadlock = true;
                                    deadlockDetails = "死锁消解失败：全局协同重规划无可行解，触发安全停机！";
                                }
                            }
                            else if (report.Status == DeadlockStatus.DeadlockConfirmed)
                            {
                                // 确认死锁时，自动尝试全局协同重规划救场
                                bool replanned = TryExecuteGlobalDeadlockReplanning("死锁确认自动解冻");
                                if (!replanned)
                                {
                                    isConfirmedDeadlock = true;
                                    if (report.DeadlockCycles.Count > 0)
                                    {
                                        var c = report.DeadlockCycles[0];
                                        deadlockDetails = $"循环依赖死锁环: 机器人 [{c.GetCycleDescription()}]，全局重规划无解！";
                                    }
                                    else
                                    {
                                        deadlockDetails = "检测到机群不可解循环等待死锁！";
                                    }
                                }
                            }
                            else if (report.Status == DeadlockStatus.TransitiveBlockage)
                            {
                                if (currentSeconds - _lastMultiProgressTime > 2.0)
                                {
                                    _lblMultiInfo.Text = "通道排队等待中 (有序推进，非死锁)";
                                    _lblMultiInfo.ForeColor = Color.FromArgb(200, 120, 30);
                                }
                            }
                        }

                        // 备用保底判定：若持续无进展超过 4.5 秒，自动触发全局重规划救场
                        double stallDuration = currentSeconds - _lastMultiProgressTime;
                        if (stallDuration > 4.5 && !hasMotion)
                        {
                            bool replanned = TryExecuteGlobalDeadlockReplanning("机群长时间停滞自动重规划救场");
                            if (!replanned)
                            {
                                isConfirmedDeadlock = true;
                                if (string.IsNullOrEmpty(deadlockDetails))
                                {
                                    deadlockDetails = "机群持续停滞且全局协同重规划无解，判定为通道刚性死锁！";
                                }
                            }
                        }

                        if (isConfirmedDeadlock || isTimeExceeded)
                        {
                            // 判定为死锁任务失败，立即安全制动并向用户反馈诊断详情
                            _finalMultiNavDuration = _multiNavStopwatch.Elapsed.TotalSeconds;
                            _isMultiNavigating = false;
                            _multiNavStopwatch.Stop();
                            for (int i = 0; i < _multiRobots.Count; i++)
                            {
                                _multiRobots[i].StopAtNextGrid(CellSizeMeters);
                            }
                            SetMultiNavControlsLocked(false);

                            string failReason = isConfirmedDeadlock ? deadlockDetails : "任务执行超时 (超出动态步长时限)";
                            _lblMultiInfo.Text = $"调度终止: {failReason}！";
                            _lblMultiInfo.ForeColor = Color.FromArgb(220, 40, 40);

                            MessageBox.Show(
                                this,
                                $"多机协同调度终止！\n\n诊断详情:\n{failReason}\n\n所有机器人已在当前栅格安全刹停。可尝试更换其它多机算法 (如 ECBS/CBS) 重新规划。",
                                "多机调度诊断",
                                MessageBoxButtons.OK,
                                MessageBoxIcon.Warning);
                        }
                    }
                }
            }

            if (hasMotion)
            {
                _skiaCanvas.Invalidate();
            }
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

            if (_currentMode == SimulationMode.SingleRobot)
            {
                // 3. 高亮显示单机机器人当前所处网格
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

                // 6. 绘制单机机器人实体
                _robot.Render(canvas, _offsetX, _offsetY, _scale, BaseCellPixels, CellSizeMeters);
            }
            else
            {
                // 3~6. 多机模式：渲染所有专属对角目标点、正交规划轨迹、多机器人实体及右上角编号角标
                DrawMultiRobotTargets(canvas, cellPixels);

                if (_chkShowPath != null && _chkShowPath.Checked)
                {
                    DrawMultiPlannedPaths(canvas, cellPixels);
                }

                for (int i = 0; i < _multiRobots.Count; i++)
                {
                    _multiRobots[i].Render(canvas, _offsetX, _offsetY, _scale, BaseCellPixels, CellSizeMeters);
                }

                DrawMultiRobotBadges(canvas, cellPixels);
            }

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

        private void DrawMinimizeButton(SKCanvas canvas, SKRect btnRect, bool isMinimized)
        {
            using (var bgPaint = new SKPaint
            {
                Color = new SKColor(228, 235, 242, 220),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            using (var borderPaint = new SKPaint
            {
                Color = new SKColor(180, 195, 210, 220),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1f,
                IsAntialias = true
            })
            using (var iconPaint = new SKPaint
            {
                Color = new SKColor(60, 80, 105),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1.6f,
                StrokeCap = SKStrokeCap.Round,
                IsAntialias = true
            })
            {
                canvas.DrawRoundRect(btnRect, 3f, 3f, bgPaint);
                canvas.DrawRoundRect(btnRect, 3f, 3f, borderPaint);

                float cx = btnRect.MidX;
                float cy = btnRect.MidY;
                float halfW = 4f;

                // 横线 (最小化状态和展开状态均绘制横线)
                canvas.DrawLine(cx - halfW, cy, cx + halfW, cy, iconPaint);

                // 最小化状态下增加竖线变成 '+'
                if (isMinimized)
                {
                    canvas.DrawLine(cx, cy - halfW, cx, cy + halfW, iconPaint);
                }
            }
        }

        private void DrawViewportHud(SKCanvas canvas)
        {
            // 在左上角使用 SkiaSharp 绘制圆角 HUD 视口参数说明卡片。
            float cardWidth = 320f;
            float cardHeight = _isViewportHudMinimized ? 32f : 116f;
            SKRect hudRect = new SKRect(12f, 12f, 12f + cardWidth, 12f + cardHeight);
            _viewportHudRect = hudRect;
            _viewportMinimizeBtnRect = new SKRect(hudRect.Right - 24f, hudRect.Top + 7f, hudRect.Right - 6f, hudRect.Top + 25f);

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
                Typeface = YaHeiTypeface
            })
            using (var titlePaint = new SKPaint
            {
                Color = new SKColor(25, 40, 55),
                TextSize = 12.5f,
                IsAntialias = true,
                Typeface = YaHeiBoldTypeface
            })
            {
                canvas.DrawRoundRect(hudRect, 6f, 6f, panelPaint);
                canvas.DrawRoundRect(hudRect, 6f, 6f, strokePaint);

                // 绘制最小化/还原按钮
                DrawMinimizeButton(canvas, _viewportMinimizeBtnRect, _isViewportHudMinimized);

                string modeDesc = (_currentMode == SimulationMode.SingleRobot)
                    ? "单机模式"
                    : "多机调度 (" + _multiRobots.Count + "台)";

                // 标题文字基线严格统一为 (20f, 32f)，在折叠和展开状态下位置绝对恒定、无任何跳动偏移，且在 32px 最小化卡片中垂直精准居中
                canvas.DrawText("30 x 30 仓储栅格地图 (" + modeDesc + ")", 20f, 32f, titlePaint);

                if (_isViewportHudMinimized)
                {
                    return;
                }

                canvas.DrawText("单格尺寸: 0.55m | 机器人离散中心对齐", 20f, 52f, textPaint);
                canvas.DrawText("视口操作: 鼠标左键拖拽平移 | 滚轮缩放", 20f, 72f, textPaint);

                if (_currentMode == SimulationMode.SingleRobot)
                {
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
                else
                {
                    canvas.DrawText("多机分布: 随机分布在独立网格，无碰撞重叠", 20f, 92f, textPaint);
                    canvas.DrawText("调度模式: 待命中 (右侧可随选数量与重分布)", 20f, 112f, textPaint);
                }
            }
        }

        private void DrawStatusMonitorHud(SKCanvas canvas, int width, int height)
        {
            // 在左下角使用 SkiaSharp 绘制紧凑美观的半透明圆角状态数据表格。
            float cardWidth = 320f;
            float cardHeight = _isStatusHudMinimized ? 32f : 176f;
            float cardX = 12f;
            float cardY = height - cardHeight - 12f;

            if (cardY < 135f)
            {
                cardY = 135f;
            }

            SKRect hudRect = new SKRect(cardX, cardY, cardX + cardWidth, cardY + cardHeight);
            _statusHudRect = hudRect;
            _statusMinimizeBtnRect = new SKRect(hudRect.Right - 24f, hudRect.Top + 6f, hudRect.Right - 6f, hudRect.Top + 24f);

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
                Typeface = YaHeiBoldTypeface
            })
            using (var keyPaint = new SKPaint
            {
                Color = new SKColor(60, 75, 95),
                TextSize = 10.8f,
                IsAntialias = true,
                Typeface = YaHeiBoldTypeface
            })
            using (var valPaint = new SKPaint
            {
                Color = new SKColor(15, 23, 42),
                TextSize = 10.5f,
                IsAntialias = true,
                Typeface = YaHeiTypeface
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

                // 2. 卡片标题与最小化/还原按钮
                string monitorTitle = (_currentMode == SimulationMode.SingleRobot)
                    ? "机器人实时状态监测 [单机模式]"
                    : "多机器人集群状态监测 [" + _multiRobots.Count + "台]";
                canvas.DrawText(monitorTitle, cardX + 10f, cardY + 20f, titlePaint);
                DrawMinimizeButton(canvas, _statusMinimizeBtnRect, _isStatusHudMinimized);

                if (_isStatusHudMinimized)
                {
                    // 最小化状态仅显示标题栏，不渲染底层表格
                    return;
                }

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

                string[,] tableRows;
                if (_currentMode == SimulationMode.SingleRobot)
                {
                    tableRows = new string[7, 4]
                    {
                        { "指标", "状态/数值", "指标", "状态/数值" },
                        { "状态", fsmText, "算法", algoText },
                        { "目标", targetVal, "朝向", orientStr },
                        { "网格", gridPos, "物理", physPos },
                        { "动作", actionText, "线速", linSpeed },
                        { "加度", linAccel, "角速", angSpeed },
                        { "角加", angAccel, "底盘", "差动(0.38m)" }
                    };
                }
                else
                {
                    string collisionStr = (_collisionCount == 0) ? "0次 (安全)" : (_collisionCount + "次 (告警!)");
                    string solveTimeStr = (_multiPlanElapsedMs < 0) ? "待求解" : (_multiPlanElapsedMs + " ms");
                    string currentAlgoName = _cmbMultiAlgorithm?.SelectedItem?.ToString() ?? "ECBS";
                    if (currentAlgoName.Contains("("))
                    {
                        currentAlgoName = currentAlgoName.Substring(0, currentAlgoName.IndexOf('(')).Trim();
                    }

                    // 指标 1：总任务完成时间 (正在导航时实时累计，导航完成后固定展示本次总耗时)
                    string totalDurationStr;
                    if (_isMultiNavigating)
                    {
                        totalDurationStr = _multiNavStopwatch.Elapsed.TotalSeconds.ToString("0.0") + "s";
                    }
                    else if (_finalMultiNavDuration > 0)
                    {
                        totalDurationStr = _finalMultiNavDuration.ToString("0.0") + "s";
                    }
                    else
                    {
                        totalDurationStr = "-";
                    }

                    // 指标 2：到达率 (已达专属目标点机器人台数 / 总台数)
                    int reachedCount = 0;
                    for (int i = 0; i < _multiRobots.Count; i++)
                    {
                        Point tgt = (i < _multiTargets.Count) ? _multiTargets[i] : Point.Empty;
                        if (_multiRobots[i].GridX == tgt.X && _multiRobots[i].GridY == tgt.Y && _multiRobots[i].NavState == RobotNavState.Idle)
                        {
                            reachedCount++;
                        }
                    }
                    double arrivalRate = (_multiRobots.Count > 0) ? (reachedCount * 100.0 / _multiRobots.Count) : 0.0;
                    string arrivalRateStr = $"{arrivalRate:0.0}% ({reachedCount}/{_multiRobots.Count})";

                    // 指标 3：每个机器人的平均等待时间 (机群总等待秒数 / 机器人台数)
                    double avgWaitTime = (_multiRobots.Count > 0) ? (_totalMultiWaitTime / _multiRobots.Count) : 0.0;
                    string avgWaitStr = avgWaitTime.ToString("0.0") + "s";

                    string navStatusStr = _isMultiNavigating
                        ? "协同行进中"
                        : (reachedCount == _multiRobots.Count && _multiRobots.Count > 0 ? "全部到达" : "就绪待命");

                    tableRows = new string[7, 4]
                    {
                        { "指标", "状态/数值", "指标", "状态/数值" },
                        { "调度算法", currentAlgoName, "机群规模", _multiRobots.Count + " 台" },
                        { "求解时间", solveTimeStr, "调度状态", navStatusStr },
                        { "完成时间", totalDurationStr, "到达率", arrivalRateStr },
                        { "平均等待", avgWaitStr, "碰撞次数", collisionStr },
                        { "目标总数", _multiTargets.Count + " 个", "线速上限", linSpeed },
                        { "加速度", linAccel, "底盘规格", "0.42x0.38m" }
                    };
                }

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
            if (e.Button == MouseButtons.Left)
            {
                // 检查是否点击了标题卡片最小化按钮 (或在最小化状态下点击卡片标题栏)
                if (_viewportMinimizeBtnRect.Contains(e.X, e.Y) || (_isViewportHudMinimized && _viewportHudRect.Contains(e.X, e.Y)))
                {
                    _isViewportHudMinimized = !_isViewportHudMinimized;
                    _skiaCanvas?.Invalidate();
                    return;
                }

                // 检查是否点击了状态监测卡片最小化按钮 (或在最小化状态下点击卡片标题栏)
                if (_statusMinimizeBtnRect.Contains(e.X, e.Y) || (_isStatusHudMinimized && _statusHudRect.Contains(e.X, e.Y)))
                {
                    _isStatusHudMinimized = !_isStatusHudMinimized;
                    _skiaCanvas?.Invalidate();
                    return;
                }
            }

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
            else
            {
                // 悬停在最小化按钮或已最小化卡片区域时显示手型光标
                bool overMinimizeBtn = _viewportMinimizeBtnRect.Contains(e.X, e.Y) ||
                                       _statusMinimizeBtnRect.Contains(e.X, e.Y) ||
                                       (_isViewportHudMinimized && _viewportHudRect.Contains(e.X, e.Y)) ||
                                       (_isStatusHudMinimized && _statusHudRect.Contains(e.X, e.Y));
                _skiaCanvas.Cursor = overMinimizeBtn ? Cursors.Hand : Cursors.Default;
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
                if (_currentMode == SimulationMode.SingleRobot)
                {
                    // 动态排除左上角标题卡片区域和左下角状态卡片区域，避免点击卡片误设置底层网格目标点
                    if (_viewportHudRect.Contains(e.X, e.Y) || _statusHudRect.Contains(e.X, e.Y))
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
