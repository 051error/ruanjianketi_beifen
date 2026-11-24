using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Windows.Forms;
using SkiaSharp;
using WarehouseGridSimulation.Core.App;
using WarehouseGridSimulation.Core.Engine;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Motion;
using WarehouseGridSimulation.Core.Planning;
using WarehouseGridSimulation.Core.Robots;
using WarehouseGridSimulation.Skia.Controls;
using WarehouseGridSimulation.Skia.Rendering;
using WarehouseGridSimulation.Skia.UI;

namespace WarehouseGridSimulation.Skia.Forms
{
    /// <summary>
    /// 主仿真窗体宿主 (MainForm)。
    /// 遵循架构职责原则：
    /// 1. WinForms 仅负责创建窗口容器、承载 SkiaCanvasControl 绘图表面、转发鼠标与键盘系统事件；
    /// 2. 窗口内部的所有地图、机器人、路径、预约高亮、HUD 监视卡片与操作按钮全部由 SkiaSharp 绘制；
    /// 3. UI 通过 SimulationApp 统一入口发送操作指令，绝不直接篡改机器人位置、任务状态或资源预约。
    /// </summary>
    public sealed class MainForm : Form
    {
        private const float BaseCellPixels = 24f;
        private const float MinimumScale = 0.35f;
        private const float MaximumScale = 3.5f;

        private float _scale = 1.0f;
        private float _offsetX = 40f;
        private float _offsetY = 80f;

        private bool _isPanning = false;
        private Point _lastMousePosition;
        private Point _mousePressLocation;
        private bool _firstLayout = true;

        private readonly SimulationApp _app;
        private readonly SkiaWarehouseRenderer _renderer;
        private readonly SkiaUiManager _uiManager;

        private SkiaCanvasControl _skiaCanvas;
        private readonly Timer _renderTimer;
        private readonly Stopwatch _stopwatch;
        private double _lastElapsedSeconds;
        private double _simTimeAccumulator = 0.0;
        private double _lastPaintSeconds = -1.0;
        private double _averageFrameSeconds;
        private bool _interpolateCurrentStep;

        // 右侧参数与操作 GUI 控制面板最小化/折叠状态管理
        private bool _isGuiPanelMinimized = false;
        private const int ControlPanelNormalWidth = 290;
        private const int ControlPanelMinimizedWidth = 36;
        private Panel _controlPanel;
        private Panel _pnlMinimizedGuiBar;
        private Button _btnMinimizeGuiPanel;
        private Button _btnExpandGuiPanel;
        private ComboBox _cmbMode;
        private SimulationMode _currentMode = SimulationMode.SingleRobot;

        // 单机控制面板控件字段
        private Panel _pnlSingleMode;
        private ComboBox _cmbAlgorithm;
        private Label _lblTargetInfo;
        private Button _btnStartNavigation;
        private Button _btnCancelNavigation;
        private Button _btnMoveForward;
        private Button _btnTurnLeft;
        private Button _btnTurnRight;
        private Button _btnResetPosition;

        // 多机控制面板控件字段
        private Panel _pnlMultiMode;
        private ComboBox _cmbRobotCount;
        private Button _btnRandomizeRobots;
        private CheckBox _chkShowTargetLines;
        private ComboBox _cmbMultiAlgorithm;
        private Panel _pnlWhcaParams;
        private NumericUpDown _numWhcaK;
        private Panel _pnlEcbsParams;
        private NumericUpDown _numEcbsW;
        private Button _btnStartMultiNav;
        private Button _btnStopMultiNav;
        private Label _lblMultiInfo;
        private bool _isMultiNavigating = false;
        private readonly Stopwatch _multiNavStopwatch = new Stopwatch();
        private readonly Random _random = new Random();

        // 通用参数控制面板控件字段
        private Panel _pnlCommonParams;
        private CheckBox _chkShowPath;
        private NumericUpDown _numSpeed;
        private TrackBar _trackSpeed;
        private NumericUpDown _numAccel;
        private TrackBar _trackAccel;
        private NumericUpDown _numTurnSpeed;
        private NumericUpDown _numTurnAccel;
        private Button _btnResetParams;

        // 键盘连续按键与互斥控制状态
        private bool _isKeyWPressed = false;
        private bool _isKeyAPressed = false;
        private bool _isKeyDPressed = false;

        // 视口与 HUD 状态管理字段
        private Point? _selectedTarget = null;
        private readonly List<Point> _multiTargets = new List<Point>();
        private bool _chkShowPathChecked = true;
        private bool _chkShowTargetLinesChecked = true;
        private bool _isViewportHudMinimized = false;
        private bool _isStatusHudMinimized = false;
        private SKRect _viewportHudRect;
        private SKRect _statusHudRect;
        private SKRect _viewportMinBtnRect;
        private SKRect _statusMinBtnRect;
        private float _renderFps;

        // Skia 按钮实例引用
        private SkiaUiButton _btnStart;
        private SkiaUiButton _btnPauseResume;
        private SkiaUiButton _btnStep;
        private SkiaUiButton _btnReset;
        private SkiaUiButton _btnAddTask;
        private SkiaUiButton _btnParamToggle;

        private bool _isParamPanelExpanded = false;
        private SkiaUiButton _btnParamClose;
        private SkiaUiButton _btnSpeedMinus;
        private SkiaUiButton _btnSpeedPlus;
        private SkiaUiButton _btnAccelMinus;
        private SkiaUiButton _btnAccelPlus;
        private SkiaUiButton _btnTurnSpeedMinus;
        private SkiaUiButton _btnTurnSpeedPlus;
        private SkiaUiButton _btnTurnAccelMinus;
        private SkiaUiButton _btnTurnAccelPlus;

        public MainForm()
        {
            // 初始化主窗体容器与核心仿真应用外观。
            _app = new SimulationApp();
            _renderer = new SkiaWarehouseRenderer();
            _uiManager = new SkiaUiManager();

            Text = "仓储多机器人 RCS 调度仿真系统 (SkiaSharp 矢量渲染)";
            Size = new Size(1260, 840);
            StartPosition = FormStartPosition.CenterScreen;
            MinimumSize = new Size(960, 640);
            DoubleBuffered = true;
            BackColor = Color.FromArgb(240, 244, 248);
            KeyPreview = true;

            InitializeControls();
            BuildSkiaButtons();

            _stopwatch = Stopwatch.StartNew();
            _lastElapsedSeconds = _stopwatch.Elapsed.TotalSeconds;

            // 约 60 FPS 渲染刷新循环
            _renderTimer = new Timer();
            _renderTimer.Interval = 16;
            _renderTimer.Tick += OnRenderTimerTick;
        }

        protected override void OnShown(EventArgs e)
        {
            // 窗体初次呈现后居中网格并启动渲染时钟。
            // e: 窗体呈现事件参数。
            base.OnShown(e);
            CenterGridInView();
            _skiaCanvas?.Focus();
            _renderTimer?.Start();
        }

        protected override void Dispose(bool disposing)
        {
            // 释放渲染时钟与渲染器持有的资源。
            if (disposing)
            {
                _renderTimer?.Dispose();
                _renderer?.Dispose();
            }
            base.Dispose(disposing);
        }

        private void InitializeControls()
        {
            // 初始化控件布局并挂载右侧控制面板与绘图画布。
            SuspendLayout();

            _controlPanel = new Panel();
            _controlPanel.SuspendLayout();
            _controlPanel.Dock = DockStyle.Right;
            _controlPanel.Width = ControlPanelNormalWidth;
            _controlPanel.BackColor = Color.White;
            _controlPanel.BorderStyle = BorderStyle.FixedSingle;
            _controlPanel.AutoScroll = true;

            BuildControlPanelContent();

            _controlPanel.ResumeLayout(false);
            _controlPanel.PerformLayout();

            _skiaCanvas = new SkiaCanvasControl();
            _skiaCanvas.Dock = DockStyle.Fill;
            _skiaCanvas.TabStop = true;
            _skiaCanvas.PreviewKeyDown += (s, e) =>
            {
                switch (e.KeyCode)
                {
                    case Keys.Up:
                    case Keys.Down:
                    case Keys.Left:
                    case Keys.Right:
                    case Keys.Space:
                        e.IsInputKey = true;
                        break;
                }
            };
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
            // 构建右侧控制面板内部的折叠条与模式选择控件。
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

            // 1.1 寻路算法选择
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

            // 1.2 目标设置与导航操作
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
            _btnStartNavigation.Cursor = Cursors.Hand;
            _btnStartNavigation.Click += delegate { HandleStartNavigation(); };
            _pnlSingleMode.Controls.Add(_btnStartNavigation);
            sY += 36;

            _btnCancelNavigation = new Button();
            _btnCancelNavigation.Text = "取消导航";
            _btnCancelNavigation.Location = new Point(14, sY);
            _btnCancelNavigation.Size = new Size(250, 28);
            _btnCancelNavigation.Font = new Font("Microsoft YaHei", 8.5f);
            _btnCancelNavigation.Cursor = Cursors.Hand;
            _btnCancelNavigation.Click += delegate { HandleCancelNavigation(); };
            _pnlSingleMode.Controls.Add(_btnCancelNavigation);
            sY += 34;

            _pnlSingleMode.Controls.Add(CreateHorizontalSeparator(sY));
            sY += 14;

            // 1.3 手动动作控制
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
            _btnMoveForward.Cursor = Cursors.Hand;
            _btnMoveForward.Click += delegate { HandleMoveForward(); };
            _pnlSingleMode.Controls.Add(_btnMoveForward);
            sY += 38;

            _btnTurnLeft = new Button();
            _btnTurnLeft.Text = "左转 90° [A / ←]";
            _btnTurnLeft.Location = new Point(14, sY);
            _btnTurnLeft.Size = new Size(120, 32);
            _btnTurnLeft.Font = new Font("Microsoft YaHei", 9f);
            _btnTurnLeft.Cursor = Cursors.Hand;
            _btnTurnLeft.Click += delegate { HandleTurnLeft(); };
            _pnlSingleMode.Controls.Add(_btnTurnLeft);

            _btnTurnRight = new Button();
            _btnTurnRight.Text = "右转 90° [D / →]";
            _btnTurnRight.Location = new Point(144, sY);
            _btnTurnRight.Size = new Size(120, 32);
            _btnTurnRight.Font = new Font("Microsoft YaHei", 9f);
            _btnTurnRight.Cursor = Cursors.Hand;
            _btnTurnRight.Click += delegate { HandleTurnRight(); };
            _pnlSingleMode.Controls.Add(_btnTurnRight);
            sY += 38;

            _btnResetPosition = new Button();
            _btnResetPosition.Text = "复位到中心 [空格]";
            _btnResetPosition.Location = new Point(14, sY);
            _btnResetPosition.Size = new Size(250, 30);
            _btnResetPosition.Font = new Font("Microsoft YaHei", 8.5f);
            _btnResetPosition.Cursor = Cursors.Hand;
            _btnResetPosition.Click += delegate { HandleResetPosition(); };
            _pnlSingleMode.Controls.Add(_btnResetPosition);
            sY += 36;

            _pnlSingleMode.Height = sY;
            _pnlSingleMode.ResumeLayout(false);
            _controlPanel.Controls.Add(_pnlSingleMode);

            // 2. 多机模式专用控制面板
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
            _btnRandomizeRobots.Cursor = Cursors.Hand;
            _btnRandomizeRobots.Click += delegate { HandleRandomizeRobots(); };
            _pnlMultiMode.Controls.Add(_btnRandomizeRobots);
            mY += 36;

            _chkShowTargetLines = new CheckBox();
            _chkShowTargetLines.Text = "开启目标配对连线";
            _chkShowTargetLines.Font = new Font("Microsoft YaHei", 9f);
            _chkShowTargetLines.Checked = true;
            _chkShowTargetLines.Location = new Point(14, mY);
            _chkShowTargetLines.Size = new Size(250, 24);
            _chkShowTargetLines.CheckedChanged += delegate
            {
                _chkShowTargetLinesChecked = _chkShowTargetLines.Checked;
                _skiaCanvas?.Invalidate();
            };
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
            _btnStartMultiNav.Cursor = Cursors.Hand;
            _btnStartMultiNav.Click += delegate { HandleStartMultiNavigation(); };
            _pnlMultiMode.Controls.Add(_btnStartMultiNav);
            mY += 36;

            _btnStopMultiNav = new Button();
            _btnStopMultiNav.Text = "停止多机调度";
            _btnStopMultiNav.Location = new Point(14, mY);
            _btnStopMultiNav.Size = new Size(250, 28);
            _btnStopMultiNav.Font = new Font("Microsoft YaHei", 8.5f);
            _btnStopMultiNav.Cursor = Cursors.Hand;
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

            // 3. 通用参数设置面板 (速度、加速度、角速度与显示设置)
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
            _chkShowPath.CheckedChanged += delegate
            {
                _chkShowPathChecked = _chkShowPath.Checked;
                _skiaCanvas?.Invalidate();
            };
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
            _numAccel.Value = 1.50m;
            _numAccel.ValueChanged += OnNumAccelValueChanged;
            _pnlCommonParams.Controls.Add(_numAccel);
            cY += 32;

            _trackAccel = new TrackBar();
            _trackAccel.Location = new Point(10, cY);
            _trackAccel.Size = new Size(255, 45);
            _trackAccel.Minimum = 20;
            _trackAccel.Maximum = 600;
            _trackAccel.TickFrequency = 60;
            _trackAccel.Value = 150;
            _trackAccel.Scroll += OnTrackAccelScroll;
            _pnlCommonParams.Controls.Add(_trackAccel);
            cY += 54;

            // 角速度与角加速度调节
            Label lblTurnSpeedTitle = new Label();
            lblTurnSpeedTitle.Text = "角速度 (°/s):";
            lblTurnSpeedTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblTurnSpeedTitle.Location = new Point(14, cY);
            lblTurnSpeedTitle.Size = new Size(130, 24);
            _pnlCommonParams.Controls.Add(lblTurnSpeedTitle);

            _numTurnSpeed = new NumericUpDown();
            _numTurnSpeed.Location = new Point(155, cY - 2);
            _numTurnSpeed.Size = new Size(105, 24);
            _numTurnSpeed.Minimum = 30;
            _numTurnSpeed.Maximum = 360;
            _numTurnSpeed.Increment = 15;
            _numTurnSpeed.Value = 180;
            _numTurnSpeed.ValueChanged += OnNumTurnSpeedValueChanged;
            _pnlCommonParams.Controls.Add(_numTurnSpeed);
            cY += 32;

            Label lblTurnAccelTitle = new Label();
            lblTurnAccelTitle.Text = "角加速度 (°/s²):";
            lblTurnAccelTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblTurnAccelTitle.Location = new Point(14, cY);
            lblTurnAccelTitle.Size = new Size(130, 24);
            _pnlCommonParams.Controls.Add(lblTurnAccelTitle);

            _numTurnAccel = new NumericUpDown();
            _numTurnAccel.Location = new Point(155, cY - 2);
            _numTurnAccel.Size = new Size(105, 24);
            _numTurnAccel.Minimum = 60;
            _numTurnAccel.Maximum = 1080;
            _numTurnAccel.Increment = 30;
            _numTurnAccel.Value = 360;
            _numTurnAccel.ValueChanged += OnNumTurnAccelValueChanged;
            _pnlCommonParams.Controls.Add(_numTurnAccel);
            cY += 36;

            _btnResetParams = new Button();
            _btnResetParams.Text = "恢复默认参数 (1.5m/s, 1.5m/s², 180°/s, 360°/s²)";
            _btnResetParams.Location = new Point(14, cY);
            _btnResetParams.Size = new Size(250, 30);
            _btnResetParams.Font = new Font("Microsoft YaHei", 8.5f);
            _btnResetParams.Cursor = Cursors.Hand;
            _btnResetParams.Click += delegate { HandleResetParams(); };
            _pnlCommonParams.Controls.Add(_btnResetParams);
            cY += 42;

            _pnlCommonParams.Height = cY;
            _pnlCommonParams.Location = new Point(0, modePanelTop + _pnlSingleMode.Height + 8);
            _pnlCommonParams.ResumeLayout(false);
            _controlPanel.Controls.Add(_pnlCommonParams);
        }

        private Panel CreateHorizontalSeparator(int top)
        {
            // 创建统一风格的水平分割线控件。
            // top: 控件距离容器顶部的 Y 轴像素坐标。
            Panel line = new Panel();
            line.Size = new Size(250, 1);
            line.Location = new Point(14, top);
            line.BackColor = Color.FromArgb(225, 230, 235);
            return line;
        }

        private void SetGuiPanelMinimized(bool minimized)
        {
            // 切换右侧控制面板的折叠收起与展开状态。
            // minimized: 是否收起控制面板。
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
            }

            _controlPanel.ResumeLayout(true);
            _skiaCanvas?.Invalidate();
        }

        private void OnMinimizedGuiBarPaint(object sender, PaintEventArgs e)
        {
            // 竖向高保真绘制折叠栏居中标题文本。
            // sender: 触发绘图事件的面板控件; e: 包含目标 GDI 设备上下文的绘图事件参数。
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

        private void OnModeSelectedIndexChanged(object sender, EventArgs e)
        {
            // 响应仿真运行模式切换。
            _currentMode = (_cmbMode.SelectedIndex == 1)
                ? SimulationMode.MultiRobot
                : SimulationMode.SingleRobot;
            UpdateModeControlsLayout();

            if (_currentMode == SimulationMode.MultiRobot)
            {
                if (_app.Engine.Robots.Count <= 1)
                {
                    int defaultCount = 20;
                    if (_cmbRobotCount != null && _cmbRobotCount.SelectedItem is int selCount)
                    {
                        defaultCount = selCount;
                    }
                    GenerateRandomRobots(defaultCount);
                }
            }
            else
            {
                _selectedTarget = null;
                _multiTargets.Clear();
                HandleResetPosition();
            }
            _skiaCanvas?.Invalidate();
        }

        private void UpdateModeControlsLayout()
        {
            // 动态调整单机与多机专有面板显隐及通用参数区纵向布局。
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

        private void OnMultiAlgorithmSelectedIndexChanged(object sender, EventArgs e)
        {
            // 响应多机调度算法切换并动态调整专属参数面板显隐。
            UpdateMultiAlgoParamsVisibility();
            _skiaCanvas?.Invalidate();
        }

        private void UpdateMultiAlgoParamsVisibility()
        {
            // 动态显隐多机调度算法专属参数微调面板并紧凑自适应排版。
            if (_cmbMultiAlgorithm == null || _pnlMultiMode == null) return;

            int selectedIndex = _cmbMultiAlgorithm.SelectedIndex;
            bool isEcbs = (selectedIndex == 1);
            bool isWhca = (selectedIndex == 3);

            if (_pnlEcbsParams != null) _pnlEcbsParams.Visible = isEcbs;
            if (_pnlWhcaParams != null) _pnlWhcaParams.Visible = isWhca;

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
            UpdateModeControlsLayout();
        }

        private void OnRobotCountSelectedIndexChanged(object sender, EventArgs e)
        {
            // 响应机器人机群数量选择变更。
            if (_cmbRobotCount.SelectedItem is int count)
            {
                GenerateRandomRobots(count);
            }
        }

        private void HandleRandomizeRobots()
        {
            // 响应重新随机生成机群与对角目标点按钮点击。
            int count = 20;
            if (_cmbRobotCount?.SelectedItem is int selCount) count = selCount;
            GenerateRandomRobots(count);
        }

        private void GenerateRandomRobots(int count)
        {
            // 在地图中随机生成互不重叠的多机机群并分配对角专属目标点。
            // count: 目标机器人数量。
            _multiTargets.Clear();
            _isMultiNavigating = false;
            _multiNavStopwatch.Reset();

            int totalCells = _app.CurrentMap.Width * _app.CurrentMap.Height;
            if (count > totalCells) count = totalCells;

            HashSet<int> usedPositions = new HashSet<int>();
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
            _skiaCanvas?.Invalidate();
        }

        private void GenerateDiagonalTargets()
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

        private void HandleStartMultiNavigation()
        {
            // 启动多机协同调度导航任务。
            if (_app.Engine.Robots.Count == 0 || _multiTargets.Count == 0)
            {
                MessageBox.Show(this, "请先生成机群与目标点！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Information);
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

            _isMultiNavigating = true;
            _multiNavStopwatch.Restart();
            _app.Engine.Start();

            if (_lblMultiInfo != null)
            {
                _lblMultiInfo.Text = $"多机协同导航执行中 ({_app.Engine.Robots.Count}台)...";
                _lblMultiInfo.ForeColor = Color.FromArgb(0, 110, 220);
            }
            _skiaCanvas?.Invalidate();
        }

        private void HandleStopMultiNavigation()
        {
            // 停止多机协同调度。
            _isMultiNavigating = false;
            _multiNavStopwatch.Stop();
            _app.Engine.Pause();

            if (_lblMultiInfo != null)
            {
                _lblMultiInfo.Text = $"多机调度已停止 (耗时 {_multiNavStopwatch.Elapsed.TotalSeconds:F1}s)";
                _lblMultiInfo.ForeColor = Color.FromArgb(200, 80, 40);
            }
            _skiaCanvas?.Invalidate();
        }

        private void OnNumSpeedValueChanged(object sender, EventArgs e)
        {
            // 响应速度数字微调框修改并同步至滑动条与机群动力学配置。
            float newSpeed = (float)_numSpeed.Value;
            AdjustConfig(cfg => cfg.MaxLinearSpeed = newSpeed);
            int trackVal = (int)Math.Round(newSpeed * 100f);
            trackVal = Math.Clamp(trackVal, _trackSpeed.Minimum, _trackSpeed.Maximum);
            if (_trackSpeed.Value != trackVal)
            {
                _trackSpeed.Value = trackVal;
            }
        }

        private void OnTrackSpeedScroll(object sender, EventArgs e)
        {
            // 响应速度滑动条拖动并同步至数字输入框。
            decimal speedVal = (decimal)_trackSpeed.Value / 100m;
            if (_numSpeed.Value != speedVal)
            {
                _numSpeed.Value = speedVal;
            }
        }

        private void OnNumAccelValueChanged(object sender, EventArgs e)
        {
            // 响应加速度数字微调框修改并同步至滑动条与机群动力学配置。
            float newAccel = (float)_numAccel.Value;
            AdjustConfig(cfg => cfg.LinearAcceleration = newAccel);
            int trackVal = (int)Math.Round(newAccel * 100f);
            trackVal = Math.Clamp(trackVal, _trackAccel.Minimum, _trackAccel.Maximum);
            if (_trackAccel.Value != trackVal)
            {
                _trackAccel.Value = trackVal;
            }
        }

        private void OnTrackAccelScroll(object sender, EventArgs e)
        {
            // 响应加速度滑动条拖动并同步至数字输入框。
            decimal accelVal = (decimal)_trackAccel.Value / 100m;
            if (_numAccel.Value != accelVal)
            {
                _numAccel.Value = accelVal;
            }
        }

        private void OnNumTurnSpeedValueChanged(object sender, EventArgs e)
        {
            // 响应角速度数字框修改并转换为弧度同步至机群配置。
            float turnSpeedRad = (float)_numTurnSpeed.Value * (float)Math.PI / 180f;
            AdjustConfig(cfg => cfg.TurnSpeed = turnSpeedRad);
        }

        private void OnNumTurnAccelValueChanged(object sender, EventArgs e)
        {
            // 响应角加速度数字框修改并转换为弧度同步至机群配置。
            float turnAccelRad = (float)_numTurnAccel.Value * (float)Math.PI / 180f;
            AdjustConfig(cfg => cfg.AngularAcceleration = turnAccelRad);
        }

        private void HandleResetParams()
        {
            // 恢复所有动力学参数至默认基准值。
            _numSpeed.Value = 1.50m;
            _numAccel.Value = 1.50m;
            _numTurnSpeed.Value = 180m;
            _numTurnAccel.Value = 360m;
        }

        private void OnAlgorithmSelectedIndexChanged(object sender, EventArgs e)
        {
            // 响应单机寻路规划算法选择变更并同步至仿真引擎。
            if (_app.Engine != null)
            {
                if (_cmbAlgorithm.SelectedIndex == 1)
                {
                    _app.Engine.PathPlanner = new DijkstraPlanner();
                }
                else
                {
                    _app.Engine.PathPlanner = new AStarPlanner();
                }
            }

            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r != null && _selectedTarget.HasValue && _app.CurrentMap != null)
            {
                var mc = _app.Engine.GetMotionController(r.Id);
                if (mc != null && (mc.IsPathCompleted || !mc.HasPath))
                {
                    var planResult = _app.Engine.PathPlanner.PlanPath(
                        _app.CurrentMap,
                        r.ConfirmedGrid,
                        new GridPoint(_selectedTarget.Value.X, _selectedTarget.Value.Y));
                    if (planResult.Success)
                    {
                        mc.SetPath(planResult.Path);
                        _app.Engine.PublishSnapshot();
                    }
                }
            }

            _skiaCanvas?.Invalidate();
        }

        public void SetSelectedTarget(Point target)
        {
            // 设置当前选定的目标网格并更新状态提示与路径规划。
            // target: 选中的离散网格坐标。
            _selectedTarget = target;
            if (_lblTargetInfo != null)
            {
                _lblTargetInfo.Text = $"目标网格: [{target.X}, {target.Y}]";
                _lblTargetInfo.ForeColor = Color.FromArgb(0, 110, 220);
            }

            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r != null && _app.Engine?.PathPlanner != null && _app.CurrentMap != null)
            {
                var mc = _app.Engine.GetMotionController(r.Id);
                if (mc != null && (mc.IsPathCompleted || !mc.HasPath))
                {
                    var planResult = _app.Engine.PathPlanner.PlanPath(
                        _app.CurrentMap,
                        r.ConfirmedGrid,
                        new GridPoint(target.X, target.Y));
                    if (planResult.Success)
                    {
                        mc.SetPath(planResult.Path);
                        _app.Engine.PublishSnapshot();
                    }
                }
            }

            _skiaCanvas?.Invalidate();
        }

        private void HandleStartNavigation()
        {
            // 启动单机自动寻路导航任务。
            if (!_selectedTarget.HasValue)
            {
                MessageBox.Show(this, "请先在地图中单击任意网格以设定目标点！", "提示", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r == null) return;

            Point target = _selectedTarget.Value;
            var planResult = _app.Engine.PathPlanner.PlanPath(_app.CurrentMap, r.ConfirmedGrid, new GridPoint(target.X, target.Y));
            if (planResult.Success)
            {
                var mc = _app.Engine.GetMotionController(r.Id);
                mc?.SetPath(planResult.Path);
                _app.Engine.Start();
                if (_lblTargetInfo != null)
                {
                    _lblTargetInfo.Text = $"导航中: [{r.ConfirmedGrid.X},{r.ConfirmedGrid.Y}] -> [{target.X},{target.Y}]";
                    _lblTargetInfo.ForeColor = Color.FromArgb(0, 140, 60);
                }
            }
            else
            {
                MessageBox.Show(this, $"未找到可行路径: {planResult.ErrorMessage}", "寻路失败", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
            _skiaCanvas?.Invalidate();
        }

        private void HandleCancelNavigation()
        {
            // 取消单机导航任务并平缓停止。
            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r != null)
            {
                var mc = _app.Engine.GetMotionController(r.Id);
                mc?.ClearPath();
            }
            _selectedTarget = null;
            if (_lblTargetInfo != null)
            {
                _lblTargetInfo.Text = "已取消导航 (停在当前网格)";
                _lblTargetInfo.ForeColor = Color.FromArgb(200, 80, 40);
            }
            _skiaCanvas?.Invalidate();
        }

        private void HandleMoveForward()
        {
            // 控制机器人向当前航向前进一个网格距离。
            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r == null) return;

            // 转向与前进严格互斥：自旋未静止前拒绝接收前进位移
            if (Math.Abs(r.AngularVelocity) > 0.02f) return;

            var mc = _app.Engine.GetMotionController(r.Id);
            mc?.ClearPath();
            r.ExecuteCommand(RobotCommand.Move(_app.CurrentMap.CellSizeMeters));
            _app.Engine.Start();
            _skiaCanvas?.Invalidate();
        }

        private void HandleTurnLeft()
        {
            // 控制机器人逆时针原地自旋 90 度。
            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r == null) return;

            // 转向与前进严格互斥：线速度未静止前拒绝接收自旋角度
            if (r.LinearVelocity > r.Config.SpeedTolerance) return;

            var mc = _app.Engine.GetMotionController(r.Id);
            mc?.ClearPath();
            r.ExecuteCommand(RobotCommand.Rotate(-(float)Math.PI * 0.5f));
            _app.Engine.Start();
            _skiaCanvas?.Invalidate();
        }

        private void HandleTurnRight()
        {
            // 控制机器人顺时针原地自旋 90 度。
            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r == null) return;

            // 转向与前进严格互斥：线速度未静止前拒绝接收自旋角度
            if (r.LinearVelocity > r.Config.SpeedTolerance) return;

            var mc = _app.Engine.GetMotionController(r.Id);
            mc?.ClearPath();
            r.ExecuteCommand(RobotCommand.Rotate((float)Math.PI * 0.5f));
            _app.Engine.Start();
            _skiaCanvas?.Invalidate();
        }

        private void HandleResetPosition()
        {
            // 将机器人复位到地图正中央。
            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r == null) return;

            var mc = _app.Engine.GetMotionController(r.Id);
            mc?.ClearPath();

            float centerCoordX = (14 + 0.5f) * _app.CurrentMap.CellSizeMeters;
            float centerCoordY = (14 + 0.5f) * _app.CurrentMap.CellSizeMeters;
            r.TeleportForReset(new Pose2D(centerCoordX, centerCoordY, -(float)Math.PI * 0.5f));
            _app.Engine.PublishSnapshot();
            _interpolateCurrentStep = false;

            _selectedTarget = null;
            if (_lblTargetInfo != null)
            {
                _lblTargetInfo.Text = "已复位到中心 (14, 14)";
                _lblTargetInfo.ForeColor = Color.FromArgb(120, 130, 140);
            }
            _skiaCanvas?.Invalidate();
        }

        private void BuildSkiaButtons()
        {
            // 构建视口工具栏控制按钮与动力学参数展开面板。
            _uiManager.Clear();

            float btnWidth = 78f;
            float btnHeight = 32f;
            float btnTop = 10f;
            float rightMargin = 16f;
            float gap = 6f;

            float canvasWidth = _skiaCanvas?.Width > 0 ? _skiaCanvas.Width : ClientSize.Width;
            float totalTopBarWidth = (btnWidth + gap) * 4 + (btnWidth + 14) + 82f;
            float startX = canvasWidth - rightMargin - totalTopBarWidth;

            _btnStart = new SkiaUiButton("▶ 启动", new SKRect(startX, btnTop, startX + btnWidth, btnTop + btnHeight), () =>
            {
                _app.Start();
                UpdatePauseResumeButtonText();
                _skiaCanvas?.Invalidate();
            })
            {
                NormalColor = new SKColor(39, 174, 96),
                HoverColor = new SKColor(46, 204, 113),
                PressedColor = new SKColor(30, 132, 73)
            };

            startX += btnWidth + gap;
            _btnPauseResume = new SkiaUiButton("⏸ 暂停", new SKRect(startX, btnTop, startX + btnWidth, btnTop + btnHeight), () =>
            {
                if (_app.Engine != null)
                {
                    if (_app.Engine.IsPaused)
                    {
                        _app.Resume();
                    }
                    else
                    {
                        _interpolateCurrentStep = false;
                        _app.Pause();
                    }
                    UpdatePauseResumeButtonText();
                    _skiaCanvas?.Invalidate();
                }
            })
            {
                NormalColor = new SKColor(230, 126, 34),
                HoverColor = new SKColor(243, 156, 18),
                PressedColor = new SKColor(186, 74, 0)
            };

            startX += btnWidth + gap;
            _btnStep = new SkiaUiButton("⏭ 单步", new SKRect(startX, btnTop, startX + btnWidth, btnTop + btnHeight), () =>
            {
                _interpolateCurrentStep = false;
                _app.Step();
                UpdatePauseResumeButtonText();
                _skiaCanvas?.Invalidate();
            })
            {
                NormalColor = new SKColor(41, 128, 185),
                HoverColor = new SKColor(52, 152, 219),
                PressedColor = new SKColor(31, 97, 141)
            };

            startX += btnWidth + gap;
            _btnReset = new SkiaUiButton("↺ 重置", new SKRect(startX, btnTop, startX + btnWidth, btnTop + btnHeight), () =>
            {
                _interpolateCurrentStep = false;
                _simTimeAccumulator = 0;
                _app.Reset();
                UpdatePauseResumeButtonText();
                _skiaCanvas?.Invalidate();
            })
            {
                NormalColor = new SKColor(192, 57, 43),
                HoverColor = new SKColor(231, 76, 60),
                PressedColor = new SKColor(146, 43, 33)
            };

            startX += btnWidth + gap;
            _btnAddTask = new SkiaUiButton("➕ 随机任务", new SKRect(startX, btnTop, startX + btnWidth + 14, btnTop + btnHeight), () =>
            {
                _app.CreateRandomTask();
                _skiaCanvas?.Invalidate();
            })
            {
                NormalColor = new SKColor(142, 68, 173),
                HoverColor = new SKColor(155, 89, 182),
                PressedColor = new SKColor(108, 52, 131)
            };

            startX += btnWidth + 14 + gap;
            _btnParamToggle = new SkiaUiButton(_isParamPanelExpanded ? "⚙ 收起" : "⚙ 参数", new SKRect(startX, btnTop, startX + 76f, btnTop + btnHeight), () =>
            {
                _isParamPanelExpanded = !_isParamPanelExpanded;
                BuildSkiaButtons();
                _skiaCanvas?.Invalidate();
            })
            {
                NormalColor = new SKColor(70, 85, 110),
                HoverColor = new SKColor(90, 105, 130),
                PressedColor = new SKColor(50, 65, 85)
            };

            _uiManager.AddButton(_btnStart);
            _uiManager.AddButton(_btnPauseResume);
            _uiManager.AddButton(_btnStep);
            _uiManager.AddButton(_btnReset);
            _uiManager.AddButton(_btnAddTask);
            _uiManager.AddButton(_btnParamToggle);

            // 如果处于展开状态，构建卡片内部的参数调节微调按钮
            if (_isParamPanelExpanded)
            {
                float panelWidth = 320f;
                float panelRight = canvasWidth - 20f;
                float panelLeft = panelRight - panelWidth;
                float panelTop = 56f;

                _btnParamClose = new SkiaUiButton("✕", new SKRect(panelLeft + 288, panelTop + 8, panelLeft + 312, panelTop + 30), () =>
                {
                    _isParamPanelExpanded = false;
                    BuildSkiaButtons();
                    _skiaCanvas?.Invalidate();
                })
                {
                    NormalColor = new SKColor(180, 70, 70),
                    HoverColor = new SKColor(210, 80, 80),
                    PressedColor = new SKColor(150, 50, 50),
                    CornerRadius = 4f,
                    FontSize = 11f
                };
                _uiManager.AddButton(_btnParamClose);

                // 行 1：最大线速度 [-] [+]
                _btnSpeedMinus = new SkiaUiButton("-", new SKRect(panelLeft + 144, panelTop + 50, panelLeft + 172, panelTop + 76), () =>
                {
                    AdjustConfig(cfg => cfg.MaxLinearSpeed = Math.Max(0.2f, (float)Math.Round(cfg.MaxLinearSpeed - 0.1f, 1)));
                }) { CornerRadius = 4f, NormalColor = new SKColor(80, 100, 125) };
                _btnSpeedPlus = new SkiaUiButton("+", new SKRect(panelLeft + 242, panelTop + 50, panelLeft + 270, panelTop + 76), () =>
                {
                    AdjustConfig(cfg => cfg.MaxLinearSpeed = Math.Min(4.0f, (float)Math.Round(cfg.MaxLinearSpeed + 0.1f, 1)));
                }) { CornerRadius = 4f, NormalColor = new SKColor(80, 100, 125) };

                // 行 2：最大线加速度 [-] [+]
                _btnAccelMinus = new SkiaUiButton("-", new SKRect(panelLeft + 144, panelTop + 88, panelLeft + 172, panelTop + 114), () =>
                {
                    AdjustConfig(cfg => cfg.LinearAcceleration = Math.Max(0.2f, (float)Math.Round(cfg.LinearAcceleration - 0.1f, 1)));
                }) { CornerRadius = 4f, NormalColor = new SKColor(80, 100, 125) };
                _btnAccelPlus = new SkiaUiButton("+", new SKRect(panelLeft + 242, panelTop + 88, panelLeft + 270, panelTop + 114), () =>
                {
                    AdjustConfig(cfg => cfg.LinearAcceleration = Math.Min(5.0f, (float)Math.Round(cfg.LinearAcceleration + 0.1f, 1)));
                }) { CornerRadius = 4f, NormalColor = new SKColor(80, 100, 125) };

                // 行 3：最大角速度 [-] [+]
                _btnTurnSpeedMinus = new SkiaUiButton("-", new SKRect(panelLeft + 144, panelTop + 126, panelLeft + 172, panelTop + 152), () =>
                {
                    AdjustConfig(cfg => cfg.TurnSpeed = Math.Max(30f * (float)Math.PI / 180f, cfg.TurnSpeed - 15f * (float)Math.PI / 180f));
                }) { CornerRadius = 4f, NormalColor = new SKColor(80, 100, 125) };
                _btnTurnSpeedPlus = new SkiaUiButton("+", new SKRect(panelLeft + 242, panelTop + 126, panelLeft + 270, panelTop + 152), () =>
                {
                    AdjustConfig(cfg => cfg.TurnSpeed = Math.Min(360f * (float)Math.PI / 180f, cfg.TurnSpeed + 15f * (float)Math.PI / 180f));
                }) { CornerRadius = 4f, NormalColor = new SKColor(80, 100, 125) };

                // 行 4：最大角加速度 [-] [+]
                _btnTurnAccelMinus = new SkiaUiButton("-", new SKRect(panelLeft + 144, panelTop + 164, panelLeft + 172, panelTop + 190), () =>
                {
                    AdjustConfig(cfg => cfg.AngularAcceleration = Math.Max(60f * (float)Math.PI / 180f, cfg.AngularAcceleration - 30f * (float)Math.PI / 180f));
                }) { CornerRadius = 4f, NormalColor = new SKColor(80, 100, 125) };
                _btnTurnAccelPlus = new SkiaUiButton("+", new SKRect(panelLeft + 242, panelTop + 164, panelLeft + 270, panelTop + 190), () =>
                {
                    AdjustConfig(cfg => cfg.AngularAcceleration = Math.Min(1080f * (float)Math.PI / 180f, cfg.AngularAcceleration + 30f * (float)Math.PI / 180f));
                }) { CornerRadius = 4f, NormalColor = new SKColor(80, 100, 125) };

                _uiManager.AddButton(_btnSpeedMinus);
                _uiManager.AddButton(_btnSpeedPlus);
                _uiManager.AddButton(_btnAccelMinus);
                _uiManager.AddButton(_btnAccelPlus);
                _uiManager.AddButton(_btnTurnSpeedMinus);
                _uiManager.AddButton(_btnTurnSpeedPlus);
                _uiManager.AddButton(_btnTurnAccelMinus);
                _uiManager.AddButton(_btnTurnAccelPlus);
            }
        }

        private void AdjustConfig(Action<RobotConfig> action)
        {
            // 实时调节当前运行机群的动力学参数配置。
            // action: 针对机器人配置的修改委托。
            if (_app.Engine != null)
            {
                foreach (var r in _app.Engine.Robots)
                {
                    action(r.Config);
                }
            }
            _skiaCanvas?.Invalidate();
        }

        private RobotConfig GetActiveConfig()
        {
            // 获取当前机群首台机器人的动力学配置引用。
            return _app.Engine?.Robots?.FirstOrDefault()?.Config;
        }

        private void UpdateButtonLayout()
        {
            // 在视口尺寸发生变化时重新构建并布局所有 Skia 交互按钮。
            BuildSkiaButtons();
        }

        private void UpdatePauseResumeButtonText()
        {
            // 根据仿真引擎运行状态动态更新启动与暂停按钮的文本与色调。
            if (_app.Engine != null)
            {
                if (_btnPauseResume != null)
                {
                    _btnPauseResume.Text = _app.Engine.IsPaused ? "▶ 继续" : "⏸ 暂停";
                }
                if (_btnStart != null)
                {
                    _btnStart.Text = _app.Engine.IsRunning ? "● 运行中" : "▶ 启动";
                    _btnStart.NormalColor = _app.Engine.IsRunning ? new SKColor(46, 117, 89) : new SKColor(39, 174, 96);
                }
            }
        }

        protected override void OnResize(EventArgs e)
        {
            // 响应窗体尺寸改变并更新按钮排布与首帧网格居中。
            base.OnResize(e);
            if (_uiManager == null || _app == null) return;
            UpdateButtonLayout();
            if (_firstLayout && ClientSize.Width > 100 && ClientSize.Height > 100)
            {
                CenterGridInView();
                _firstLayout = false;
            }
        }

        private void CenterGridInView()
        {
            // 将 30x30 栅格居中对齐在当前视口中央。
            if (_app?.CurrentMap == null) return;
            float cellPixels = BaseCellPixels * _scale;
            float gridWidth = _app.CurrentMap.Width * cellPixels;
            float gridHeight = _app.CurrentMap.Height * cellPixels;

            _offsetX = (ClientSize.Width - gridWidth) * 0.5f;
            _offsetY = Math.Max(65f, (ClientSize.Height - gridHeight) * 0.5f);
        }

        private void OnRenderTimerTick(object sender, EventArgs e)
        {
            // 响应固定帧率渲染定时器并驱动物理步长积分。
            // sender: 触发事件的对象源; e: 定时器事件参数。
            double now = _stopwatch.Elapsed.TotalSeconds;
            double elapsed = now - _lastElapsedSeconds;
            _lastElapsedSeconds = now;

            if (elapsed > 0.2) elapsed = 0.2; // 避免切后台大卡顿时跳步过多

            // 单机模式下按住键盘连续移动信号持续维护
            if (_currentMode == SimulationMode.SingleRobot)
            {
                var r0 = _app.Engine?.Robots?.FirstOrDefault();
                if (r0 != null)
                {
                    if (_isKeyWPressed && Math.Abs(r0.AngularVelocity) <= 0.02f)
                    {
                        var mc = _app.Engine.GetMotionController(r0.Id);
                        mc?.ClearPath();
                        r0.ExecuteCommand(RobotCommand.Move(100f));
                        if (!_app.Engine.IsRunning || _app.Engine.IsPaused) _app.Engine.Start();
                    }
                    else if (_isKeyAPressed && r0.LinearVelocity <= r0.Config.SpeedTolerance)
                    {
                        var mc = _app.Engine.GetMotionController(r0.Id);
                        mc?.ClearPath();
                        r0.ExecuteCommand(RobotCommand.Rotate(-10f));
                        if (!_app.Engine.IsRunning || _app.Engine.IsPaused) _app.Engine.Start();
                    }
                    else if (_isKeyDPressed && r0.LinearVelocity <= r0.Config.SpeedTolerance)
                    {
                        var mc = _app.Engine.GetMotionController(r0.Id);
                        mc?.ClearPath();
                        r0.ExecuteCommand(RobotCommand.Rotate(10f));
                        if (!_app.Engine.IsRunning || _app.Engine.IsPaused) _app.Engine.Start();
                    }
                }
            }

            // 驱动仿真物理时钟步长推进
            if (_app.Engine != null && _app.Engine.IsRunning && !_app.Engine.IsPaused)
            {
                _simTimeAccumulator += elapsed;
                float fixedDt = _app.Engine.Config.FixedDeltaTime;

                while (_simTimeAccumulator >= fixedDt && !_app.Engine.IsPaused)
                {
                    _app.Engine.Step();
                    _simTimeAccumulator -= fixedDt;
                    _interpolateCurrentStep = !_app.Engine.IsPaused;
                    if (_app.Engine.IsPaused)
                    {
                        _simTimeAccumulator = 0;
                    }
                }
            }
            else
            {
                _interpolateCurrentStep = false;
            }

            // 单机模式下同步导航状态与按钮可用性
            if (_currentMode == SimulationMode.SingleRobot)
            {
                var r0 = _app.Engine?.Robots?.FirstOrDefault();
                if (r0 != null)
                {
                    var mc0 = _app.Engine.GetMotionController(r0.Id);
                    bool isMoving = mc0 != null && mc0.HasPath && !mc0.IsPathCompleted;
                    if (_btnStartNavigation != null) _btnStartNavigation.Enabled = !isMoving;
                    if (_cmbAlgorithm != null) _cmbAlgorithm.Enabled = !isMoving;

                    if (isMoving && _selectedTarget.HasValue && _lblTargetInfo != null)
                    {
                        _lblTargetInfo.Text = $"自动导航中 -> [{_selectedTarget.Value.X}, {_selectedTarget.Value.Y}]";
                        _lblTargetInfo.ForeColor = Color.FromArgb(0, 110, 220);
                    }
                    else if (!isMoving && _selectedTarget.HasValue && r0.ConfirmedGrid.X == _selectedTarget.Value.X && r0.ConfirmedGrid.Y == _selectedTarget.Value.Y)
                    {
                        if (_lblTargetInfo != null && !_lblTargetInfo.Text.StartsWith("已到达"))
                        {
                            _lblTargetInfo.Text = $"已到达目标点 [{_selectedTarget.Value.X}, {_selectedTarget.Value.Y}]";
                            _lblTargetInfo.ForeColor = Color.FromArgb(40, 160, 80);
                        }
                    }
                }
            }
            else if (_currentMode == SimulationMode.MultiRobot && _isMultiNavigating)
            {
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
                if (allReached && _app.Engine.Robots.Count > 0)
                {
                    _isMultiNavigating = false;
                    _multiNavStopwatch.Stop();
                    if (_lblMultiInfo != null)
                    {
                        _lblMultiInfo.Text = $"全部机群已到达专属目标点！(耗时 {_multiNavStopwatch.Elapsed.TotalSeconds:F1}s)";
                        _lblMultiInfo.ForeColor = Color.FromArgb(40, 160, 80);
                    }
                }
            }

            UpdatePauseResumeButtonText();
            _skiaCanvas?.Invalidate();
        }

        private void OnPaintSkiaSurface(SKCanvas canvas, int width, int height)
        {
            // 响应 SkiaCanvasControl 画布重绘并将仿真画面绘制到表面。
            // canvas: 渲染目标画布; width: 视口宽度; height: 视口高度。
            var snapshot = _app.CurrentSnapshot;
            double paintSeconds = _stopwatch.Elapsed.TotalSeconds;
            if (_lastPaintSeconds >= 0 && paintSeconds > _lastPaintSeconds)
            {
                double frameSeconds = paintSeconds - _lastPaintSeconds;
                _averageFrameSeconds = _averageFrameSeconds == 0
                    ? frameSeconds
                    : _averageFrameSeconds * 0.9 + frameSeconds * 0.1;
                _renderFps = (float)(1.0 / _averageFrameSeconds);
            }
            _lastPaintSeconds = paintSeconds;

            float interpolationAlpha = _interpolateCurrentStep && snapshot != null &&
                snapshot.IsRunning && !snapshot.IsPaused
                ? Math.Clamp((float)(_simTimeAccumulator / _app.Engine.Config.FixedDeltaTime), 0f, 1f)
                : 1f;
            _renderer.Render(
                canvas,
                width,
                height,
                _app.CurrentMap,
                snapshot,
                _offsetX,
                _offsetY,
                _scale,
                BaseCellPixels,
                _currentMode,
                _selectedTarget,
                _chkShowPathChecked,
                _chkShowTargetLinesChecked,
                _multiTargets,
                _isViewportHudMinimized,
                _isStatusHudMinimized,
                ref _viewportHudRect,
                ref _statusHudRect,
                ref _viewportMinBtnRect,
                ref _statusMinBtnRect,
                _renderFps,
                interpolationAlpha);
        }

        // ==================== 鼠标事件转发与命中测试 ====================

        private void OnCanvasMouseDown(object sender, MouseEventArgs e)
        {
            // 响应鼠标按下事件并执行 HUD 折叠按钮与 UI 命中测试或启动视口平移。
            // e: 鼠标事件参数。
            _skiaCanvas?.Focus();
            _mousePressLocation = e.Location;

            // 1. 优先检测是否点击了左上角 Viewport HUD 或左下角 Status HUD 的最小化折叠按钮
            if (_viewportMinBtnRect.Contains(e.X, e.Y))
            {
                _isViewportHudMinimized = !_isViewportHudMinimized;
                _skiaCanvas?.Invalidate();
                return;
            }
            if (_statusMinBtnRect.Contains(e.X, e.Y))
            {
                _isStatusHudMinimized = !_isStatusHudMinimized;
                _skiaCanvas?.Invalidate();
                return;
            }

            // 2. 如果点击落在 HUD 卡片主体区域内，拦截后续平移与网格拾取
            if (_viewportHudRect.Contains(e.X, e.Y) || _statusHudRect.Contains(e.X, e.Y))
            {
                return;
            }

            if (_uiManager.OnMouseDown(e.X, e.Y))
            {
                _skiaCanvas?.Invalidate();
                return;
            }

            // 3. 未击中 UI 控件，则启动视口平移
            if (e.Button == MouseButtons.Left || e.Button == MouseButtons.Middle)
            {
                _isPanning = true;
                _lastMousePosition = e.Location;
            }
        }

        private void OnCanvasMouseMove(object sender, MouseEventArgs e)
        {
            // 响应鼠标移动事件并更新按钮悬停状态或视口平移偏移。
            bool uiHandled = _uiManager.OnMouseMove(e.X, e.Y);

            if (_isPanning)
            {
                int dx = e.X - _lastMousePosition.X;
                int dy = e.Y - _lastMousePosition.Y;
                _offsetX += dx;
                _offsetY += dy;
                _lastMousePosition = e.Location;
                _skiaCanvas?.Invalidate();
            }
            else if (uiHandled)
            {
                _skiaCanvas?.Invalidate();
            }
        }

        private void OnCanvasMouseUp(object sender, MouseEventArgs e)
        {
            // 响应鼠标抬起事件并触发按钮点击、网格点选或结束视口平移。
            _isPanning = false;

            // 1. 如果点击落在 HUD 区域内，直接返回
            if (_viewportHudRect.Contains(e.X, e.Y) || _statusHudRect.Contains(e.X, e.Y))
            {
                return;
            }

            if (_uiManager.OnMouseUp(e.X, e.Y))
            {
                _skiaCanvas?.Invalidate();
                return;
            }

            // 2. 单机模式下单击未发生拖拽时进行目标网格拾取
            int dragDistX = Math.Abs(e.X - _mousePressLocation.X);
            int dragDistY = Math.Abs(e.Y - _mousePressLocation.Y);
            if (dragDistX < 4 && dragDistY < 4 && _currentMode == SimulationMode.SingleRobot && _app.CurrentMap != null)
            {
                float cellPixels = BaseCellPixels * _scale;
                int gx = (int)Math.Floor((e.X - _offsetX) / cellPixels);
                int gy = (int)Math.Floor((e.Y - _offsetY) / cellPixels);

                if (gx >= 0 && gx < _app.CurrentMap.Width && gy >= 0 && gy < _app.CurrentMap.Height)
                {
                    if (!_app.CurrentMap.IsObstacle(new GridPoint(gx, gy)))
                    {
                        SetSelectedTarget(new Point(gx, gy));
                    }
                }
            }
        }

        private void OnCanvasMouseWheel(object sender, MouseEventArgs e)
        {
            // 响应鼠标滚轮滚动并以光标位置为锚点执行平滑缩放。
            float zoomFactor = e.Delta > 0 ? 1.15f : 0.87f;
            float newScale = Math.Clamp(_scale * zoomFactor, MinimumScale, MaximumScale);

            if (Math.Abs(newScale - _scale) > 0.001f)
            {
                // 以当前鼠标指针为锚点进行平滑缩放
                float mouseX = e.X;
                float mouseY = e.Y;

                _offsetX = mouseX - (mouseX - _offsetX) * (newScale / _scale);
                _offsetY = mouseY - (mouseY - _offsetY) * (newScale / _scale);
                _scale = newScale;

                _skiaCanvas?.Invalidate();
            }
        }

        // ==================== 快捷键与连续按键支持 ====================

        protected override void OnKeyDown(KeyEventArgs e)
        {
            // 响应键盘按下事件并转换为定量前进与自旋控制信号。
            // e: 键盘事件参数。
            base.OnKeyDown(e);
            if (_currentMode != SimulationMode.SingleRobot) return;

            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r == null) return;

            switch (e.KeyCode)
            {
                case Keys.W:
                case Keys.Up:
                    // 互斥检查：旋转自旋未静止前拒绝接收前进信号
                    if (Math.Abs(r.AngularVelocity) <= 0.02f)
                    {
                        _isKeyWPressed = true;
                    }
                    break;
                case Keys.A:
                case Keys.Left:
                    // 互斥检查：前进线速度未静止前拒绝接收自旋信号
                    if (r.LinearVelocity <= r.Config.SpeedTolerance)
                    {
                        _isKeyAPressed = true;
                    }
                    break;
                case Keys.D:
                case Keys.Right:
                    // 互斥检查：前进线速度未静止前拒绝接收自旋信号
                    if (r.LinearVelocity <= r.Config.SpeedTolerance)
                    {
                        _isKeyDPressed = true;
                    }
                    break;
                case Keys.Space:
                    HandleResetPosition();
                    break;
            }
        }

        protected override void OnKeyUp(KeyEventArgs e)
        {
            // 响应键盘松开事件并立即下发停止制动信号。
            // e: 键盘事件参数。
            base.OnKeyUp(e);
            if (_currentMode != SimulationMode.SingleRobot) return;

            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r == null) return;

            switch (e.KeyCode)
            {
                case Keys.W:
                case Keys.Up:
                    _isKeyWPressed = false;
                    r.ExecuteCommand(RobotCommand.Brake());
                    break;
                case Keys.A:
                case Keys.Left:
                    _isKeyAPressed = false;
                    r.ExecuteCommand(RobotCommand.Brake());
                    break;
                case Keys.D:
                case Keys.Right:
                    _isKeyDPressed = false;
                    r.ExecuteCommand(RobotCommand.Brake());
                    break;
            }
        }

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            // 拦截并处理全局仿真控制快捷键。
            // msg: Windows 消息对象; keyData: 触发的按键键码数据。
            switch (keyData)
            {
                case Keys.S:
                    _interpolateCurrentStep = false;
                    _app.Step();
                    return true;
                case Keys.R:
                    _interpolateCurrentStep = false;
                    _simTimeAccumulator = 0;
                    _app.Reset();
                    return true;
                case Keys.T:
                    _app.CreateRandomTask();
                    return true;
            }
            return base.ProcessCmdKey(ref msg, keyData);
        }
    }
}
