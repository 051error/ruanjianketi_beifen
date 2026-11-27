using System;
using System.Drawing;
using System.Windows.Forms;
using WarehouseGridSimulation.Core.Planning;
using WarehouseGridSimulation.Skia.Controls;
using WarehouseGridSimulation.Skia.Rendering;
using WarehouseGridSimulation.Skia.UI;

namespace WarehouseGridSimulation.Skia.Forms
{
    public sealed partial class MainForm
    {
        private const int ControlPanelNormalWidth = 290;
        private const int ControlPanelMinimizedWidth = 36;
        private bool _isGuiPanelMinimized = false;

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
        private CheckBox _chkEndlessMode;
        private ComboBox _cmbMultiAlgorithm;
        private Panel _pnlWhcaParams;
        private NumericUpDown _numWhcaK;
        private Panel _pnlEcbsParams;
        private NumericUpDown _numEcbsW;
        private Button _btnStartMultiNav;
        private Button _btnStopMultiNav;
        private Label _lblMultiInfo;

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

            _singleRobotPresenter = new SingleRobotPresenter(
                _app,
                _lblTargetInfo,
                _btnStartNavigation,
                _cmbAlgorithm);

            _multiRobotPresenter = new MultiRobotPresenter(
                _app,
                _lblMultiInfo);

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
            _btnMoveForward.Text = "前进 [长按 W / ↑]";
            _btnMoveForward.Location = new Point(14, sY);
            _btnMoveForward.Size = new Size(250, 32);
            _btnMoveForward.BackColor = Color.FromArgb(235, 245, 255);
            _btnMoveForward.Font = new Font("Microsoft YaHei", 9f);
            _btnMoveForward.Cursor = Cursors.Hand;
            _btnMoveForward.MouseDown += (s, e) => { if (e.Button == MouseButtons.Left) SetManualForward(true); };
            _btnMoveForward.MouseUp += (s, e) => { if (e.Button == MouseButtons.Left) SetManualForward(false); };
            _btnMoveForward.MouseLeave += (s, e) => { SetManualForward(false); };
            _pnlSingleMode.Controls.Add(_btnMoveForward);
            sY += 38;

            _btnTurnLeft = new Button();
            _btnTurnLeft.Text = "左转 [长按 A / ←]";
            _btnTurnLeft.Location = new Point(14, sY);
            _btnTurnLeft.Size = new Size(120, 32);
            _btnTurnLeft.Font = new Font("Microsoft YaHei", 9f);
            _btnTurnLeft.Cursor = Cursors.Hand;
            _btnTurnLeft.MouseDown += (s, e) => { if (e.Button == MouseButtons.Left) SetManualTurnLeft(true); };
            _btnTurnLeft.MouseUp += (s, e) => { if (e.Button == MouseButtons.Left) SetManualTurnLeft(false); };
            _btnTurnLeft.MouseLeave += (s, e) => { SetManualTurnLeft(false); };
            _pnlSingleMode.Controls.Add(_btnTurnLeft);

            _btnTurnRight = new Button();
            _btnTurnRight.Text = "右转 [长按 D / →]";
            _btnTurnRight.Location = new Point(144, sY);
            _btnTurnRight.Size = new Size(120, 32);
            _btnTurnRight.Font = new Font("Microsoft YaHei", 9f);
            _btnTurnRight.Cursor = Cursors.Hand;
            _btnTurnRight.MouseDown += (s, e) => { if (e.Button == MouseButtons.Left) SetManualTurnRight(true); };
            _btnTurnRight.MouseUp += (s, e) => { if (e.Button == MouseButtons.Left) SetManualTurnRight(false); };
            _btnTurnRight.MouseLeave += (s, e) => { SetManualTurnRight(false); };
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
            mY += 26;

            _chkEndlessMode = new CheckBox();
            _chkEndlessMode.Text = "开启无尽模式 (持续对角巡航)";
            _chkEndlessMode.Font = new Font("Microsoft YaHei", 9f);
            _chkEndlessMode.Checked = false;
            _chkEndlessMode.ForeColor = Color.FromArgb(180, 80, 0);
            _chkEndlessMode.Location = new Point(14, mY);
            _chkEndlessMode.Size = new Size(250, 24);
            _chkEndlessMode.CheckedChanged += delegate
            {
                if (_multiRobotPresenter != null)
                {
                    _multiRobotPresenter.IsEndlessMode = _chkEndlessMode.Checked;
                }
            };
            _pnlMultiMode.Controls.Add(_chkEndlessMode);
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

        private void UpdateMultiAlgoParamsVisibility()
        {
            // 动态显隐多机调度算法专属参数微调面板并紧凑自适应排版。
            if (_cmbMultiAlgorithm == null || _pnlMultiMode == null) return;

            string selectedId = (_cmbMultiAlgorithm.SelectedItem as IMultiAgentPathPlanner)?.Id ?? string.Empty;
            bool isEcbs = string.Equals(selectedId, "ECBS", StringComparison.OrdinalIgnoreCase);
            bool isWhca = string.Equals(selectedId, "WHCAStar", StringComparison.OrdinalIgnoreCase);

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
    }
}
