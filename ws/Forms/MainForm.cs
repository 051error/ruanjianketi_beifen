using System;
using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;
using WarehouseGridSimulation.Models;

namespace WarehouseGridSimulation
{
    /// <summary>
    /// 仓储物流多智能体调度仿真主窗体。
    /// 包含离散 30x30 栅格渲染、机器人在网格之间的移动控制，以及速度与加速度参数调节 GUI 面板。
    /// </summary>
    internal sealed class MainForm : Form
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
        private bool _centerGridOnFirstResize = true;

        private readonly Robot _robot;
        private readonly Timer _simulationTimer;
        private readonly Stopwatch _stopwatch;
        private double _lastElapsedSeconds;

        // 右侧 GUI 控制面板及其控件
        private Panel _controlPanel;
        private NumericUpDown _numSpeed;
        private TrackBar _trackSpeed;
        private NumericUpDown _numAccel;
        private TrackBar _trackAccel;
        private Label _lblStatusInfo;
        private Button _btnMoveForward;
        private Button _btnTurnLeft;
        private Button _btnTurnRight;
        private Button _btnResetParams;
        private Button _btnResetPosition;

        /// <summary>
        /// 初始化主窗体，构造 GDI+ 视图、机器人实体、仿真循环与参数配置 GUI 面板。
        /// </summary>
        public MainForm()
        {
            Text = "仓储物流多智能体调度仿真 - 离散栅格与机器人控制";
            Size = new Size(1180, 800);
            StartPosition = FormStartPosition.CenterScreen;
            MinimumSize = new Size(800, 600);
            DoubleBuffered = true;
            BackColor = Color.FromArgb(245, 247, 250);
            ResizeRedraw = true;
            KeyPreview = true;

            // 机器人初始位于网格中心格子 (14, 14)，朝向上方
            _robot = new Robot(14, 14, CellSizeMeters, RobotOrientation.Up);

            InitializeGuiControls();

            // 仿真驱动时钟
            _stopwatch = Stopwatch.StartNew();
            _lastElapsedSeconds = _stopwatch.Elapsed.TotalSeconds;

            _simulationTimer = new Timer();
            _simulationTimer.Interval = 16; // ~60 FPS
            _simulationTimer.Tick += OnSimulationTick;
            _simulationTimer.Start();

            MouseDown += OnMouseDown;
            MouseMove += OnMouseMove;
            MouseUp += OnMouseUp;
            MouseWheel += OnMouseWheel;
        }

        /// <summary>
        /// 创建并布局右侧的速度、加速度调节面板及状态展示控件。
        /// 控件间预留充足的纵向留白，避免在不同 DPI 缩放下产生标签或滑动条重叠遮挡。
        /// </summary>
        private void InitializeGuiControls()
        {
            _controlPanel = new Panel();
            _controlPanel.Dock = DockStyle.Right;
            _controlPanel.Width = 290;
            _controlPanel.BackColor = Color.FromArgb(255, 255, 255);
            _controlPanel.BorderStyle = BorderStyle.FixedSingle;
            _controlPanel.AutoScroll = true;

            int currentY = 16;

            // 标题
            Label lblTitle = new Label();
            lblTitle.Text = "机器人参数调节与控制";
            lblTitle.Font = new Font("Microsoft YaHei", 10.5f, FontStyle.Bold);
            lblTitle.ForeColor = Color.FromArgb(30, 45, 60);
            lblTitle.Location = new Point(14, currentY);
            lblTitle.Size = new Size(255, 24);
            _controlPanel.Controls.Add(lblTitle);
            currentY += 36;

            // 分割线 1
            Panel line1 = CreateHorizontalSeparator(currentY);
            _controlPanel.Controls.Add(line1);
            currentY += 16;

            // 1. 速度标签与数字微调框
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

            // 速度滑动条（高度预留充足，下方拉开留白）
            _trackSpeed = new TrackBar();
            _trackSpeed.Location = new Point(10, currentY);
            _trackSpeed.Size = new Size(255, 45);
            _trackSpeed.Minimum = 20; // 0.20 m/s
            _trackSpeed.Maximum = 400; // 4.00 m/s
            _trackSpeed.TickFrequency = 40;
            _trackSpeed.Value = 150; // 1.50 m/s
            _trackSpeed.Scroll += OnTrackSpeedScroll;
            _controlPanel.Controls.Add(_trackSpeed);
            currentY += 56; // 拉开与下方加速度标签的距离，彻底避免重叠遮挡

            // 2. 加速度标签与数字微调框
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

            // 加速度滑动条（高度预留充足，下方拉开留白）
            _trackAccel = new TrackBar();
            _trackAccel.Location = new Point(10, currentY);
            _trackAccel.Size = new Size(255, 45);
            _trackAccel.Minimum = 20; // 0.20 m/s²
            _trackAccel.Maximum = 600; // 6.00 m/s²
            _trackAccel.TickFrequency = 60;
            _trackAccel.Value = 200; // 2.00 m/s²
            _trackAccel.Scroll += OnTrackAccelScroll;
            _controlPanel.Controls.Add(_trackAccel);
            currentY += 56; // 拉开与下方恢复默认按钮的距离，彻底避免遮挡

            // 恢复默认参数按钮
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
            currentY += 44;

            // 分割线 2
            Panel line2 = CreateHorizontalSeparator(currentY);
            _controlPanel.Controls.Add(line2);
            currentY += 16;

            // 3. 控制动作按钮区域
            Label lblActions = new Label();
            lblActions.Text = "动作指令 (或使用键盘):";
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
            _btnMoveForward.Font = new Font("Microsoft YaHei", 9f, FontStyle.Regular);
            _btnMoveForward.Click += delegate { HandleMoveForward(); };
            _controlPanel.Controls.Add(_btnMoveForward);
            currentY += 38;

            _btnTurnLeft = new Button();
            _btnTurnLeft.Text = "左转 90° [A / ←]";
            _btnTurnLeft.Location = new Point(14, currentY);
            _btnTurnLeft.Size = new Size(120, 32);
            _btnTurnLeft.Font = new Font("Microsoft YaHei", 9f, FontStyle.Regular);
            _btnTurnLeft.Click += delegate { HandleTurnLeft(); };
            _controlPanel.Controls.Add(_btnTurnLeft);

            _btnTurnRight = new Button();
            _btnTurnRight.Text = "右转 90° [D / →]";
            _btnTurnRight.Location = new Point(144, currentY);
            _btnTurnRight.Size = new Size(120, 32);
            _btnTurnRight.Font = new Font("Microsoft YaHei", 9f, FontStyle.Regular);
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
            currentY += 42;

            // 分割线 3
            Panel line3 = CreateHorizontalSeparator(currentY);
            _controlPanel.Controls.Add(line3);
            currentY += 16;

            // 4. 状态实时展示卡片
            Label lblMonitorTitle = new Label();
            lblMonitorTitle.Text = "实时状态监测:";
            lblMonitorTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblMonitorTitle.Location = new Point(14, currentY);
            lblMonitorTitle.Size = new Size(250, 24);
            _controlPanel.Controls.Add(lblMonitorTitle);
            currentY += 28;

            _lblStatusInfo = new Label();
            _lblStatusInfo.Location = new Point(14, currentY);
            _lblStatusInfo.Size = new Size(250, 160);
            _lblStatusInfo.Font = new Font("Consolas", 9f, FontStyle.Regular);
            _lblStatusInfo.BackColor = Color.FromArgb(246, 248, 250);
            _lblStatusInfo.BorderStyle = BorderStyle.FixedSingle;
            _lblStatusInfo.Padding = new Padding(6);
            _controlPanel.Controls.Add(_lblStatusInfo);

            Controls.Add(_controlPanel);
        }

        /// <summary>
        /// 创建水平灰色分割线条。
        /// </summary>
        /// <param name="top">顶部 Y 坐标。</param>
        /// <returns>分割线 Panel。</returns>
        private Panel CreateHorizontalSeparator(int top)
        {
            Panel line = new Panel();
            line.Size = new Size(250, 1);
            line.Location = new Point(12, top);
            line.BackColor = Color.FromArgb(225, 230, 235);
            return line;
        }

        /// <summary>
        /// 当速度数字框数值变化时，同步至机器人物理参数与滑动条。
        /// </summary>
        /// <param name="sender">事件源。</param>
        /// <param name="e">事件参数。</param>
        private void OnNumSpeedValueChanged(object sender, EventArgs e)
        {
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

        /// <summary>
        /// 当速度滑动条拖动时，同步至数字框。
        /// </summary>
        /// <param name="sender">事件源。</param>
        /// <param name="e">事件参数。</param>
        private void OnTrackSpeedScroll(object sender, EventArgs e)
        {
            decimal speedVal = (decimal)_trackSpeed.Value / 100m;
            if (_numSpeed.Value != speedVal)
            {
                _numSpeed.Value = speedVal;
            }
        }

        /// <summary>
        /// 当加速度数字框数值变化时，同步至机器人物理参数与滑动条。
        /// </summary>
        /// <param name="sender">事件源。</param>
        /// <param name="e">事件参数。</param>
        private void OnNumAccelValueChanged(object sender, EventArgs e)
        {
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

        /// <summary>
        /// 当加速度滑动条拖动时，同步至数字框。
        /// </summary>
        /// <param name="sender">事件源。</param>
        /// <param name="e">事件参数。</param>
        private void OnTrackAccelScroll(object sender, EventArgs e)
        {
            decimal accelVal = (decimal)_trackAccel.Value / 100m;
            if (_numAccel.Value != accelVal)
            {
                _numAccel.Value = accelVal;
            }
        }

        /// <summary>
        /// 处理向前走一格动作。
        /// </summary>
        private void HandleMoveForward()
        {
            _robot.TryMoveForward(GridColumns, GridRows, CellSizeMeters);
        }

        /// <summary>
        /// 处理向左逆时针旋转 90 度动作。
        /// </summary>
        private void HandleTurnLeft()
        {
            _robot.TryTurnLeft();
        }

        /// <summary>
        /// 处理向右顺时针旋转 90 度动作。
        /// </summary>
        private void HandleTurnRight()
        {
            _robot.TryTurnRight();
        }

        /// <summary>
        /// 重置机器人回地图中心格子 (14, 14)。
        /// </summary>
        private void HandleResetPosition()
        {
            _robot.ResetToGrid(14, 14, CellSizeMeters);
        }

        /// <summary>
        /// 重写快捷键预处理逻辑，保证焦点在任意控件时均可通过键盘控制机器人。
        /// </summary>
        /// <param name="msg">Windows 消息。</param>
        /// <param name="keyData">按键数据。</param>
        /// <returns>若已处理该快捷键则返回 true。</returns>
        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
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

            return base.ProcessCmdKey(ref msg, keyData);
        }

        /// <summary>
        /// 仿真时钟 Tick 事件，更新物理位移与旋转，并刷新界面。
        /// </summary>
        /// <param name="sender">事件源。</param>
        /// <param name="e">事件参数。</param>
        private void OnSimulationTick(object sender, EventArgs e)
        {
            double currentSeconds = _stopwatch.Elapsed.TotalSeconds;
            float deltaTime = (float)(currentSeconds - _lastElapsedSeconds);
            _lastElapsedSeconds = currentSeconds;

            if (deltaTime > 0.1f)
            {
                deltaTime = 0.1f;
            }

            _robot.Update(deltaTime, CellSizeMeters);
            UpdateStatusLabel();
            Invalidate();
        }

        /// <summary>
        /// 刷新右侧状态面板上的文本数据。
        /// </summary>
        private void UpdateStatusLabel()
        {
            string stateText = "静止 (空闲)";
            if (_robot.State == RobotState.Moving) stateText = "移动中 -> 邻格";
            else if (_robot.State == RobotState.Turning) stateText = "旋转中 (90°)";

            string orientText = "正上 (↑)";
            switch (_robot.Orientation)
            {
                case RobotOrientation.Right: orientText = "正右 (→)"; break;
                case RobotOrientation.Down: orientText = "正下 (↓)"; break;
                case RobotOrientation.Left: orientText = "正左 (←)"; break;
            }

            string text =
                "网格坐标: [" + _robot.GridX.ToString() + ", " + _robot.GridY.ToString() + "]\r\n" +
                "物理坐标: (" + _robot.PositionX.ToString("0.00") + "m, " + _robot.PositionY.ToString("0.00") + "m)\r\n" +
                "当前朝向: " + orientText + "\r\n" +
                "实时角度: " + _robot.CurrentAngle.ToString("0.0") + "°\r\n" +
                "动作状态: " + stateText + "\r\n" +
                "瞬时速度: " + _robot.CurrentSpeed.ToString("0.00") + " m/s\r\n" +
                "设定速度: " + _robot.MaxSpeed.ToString("0.00") + " m/s\r\n" +
                "设定加度: " + _robot.Acceleration.ToString("0.00") + " m/s²";

            _lblStatusInfo.Text = text;
        }

        /// <summary>
        /// 释放由窗体占用的托管资源。
        /// </summary>
        /// <param name="disposing">是否释放托管资源。</param>
        protected override void Dispose(bool disposing)
        {
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

        /// <summary>
        /// 触发绘制事件，渲染 30x30 栅格与机器人。
        /// </summary>
        /// <param name="e">绘图参数。</param>
        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);

            // 获取排除右侧控制面板后的地图绘制可视区域
            Rectangle viewRect = new Rectangle(
                0,
                0,
                Math.Max(1, ClientRectangle.Width - _controlPanel.Width),
                ClientRectangle.Height);

            PaintScene(e.Graphics, viewRect);
        }

        /// <summary>
        /// 首次窗体布局完成后，自动将地图居中展示在左侧视图区域。
        /// </summary>
        /// <param name="e">事件参数。</param>
        protected override void OnResize(EventArgs e)
        {
            base.OnResize(e);

            int viewWidth = ClientRectangle.Width - (_controlPanel != null ? _controlPanel.Width : 0);
            if (_centerGridOnFirstResize && viewWidth > 0 && ClientRectangle.Height > 0)
            {
                CenterGrid(viewWidth, ClientRectangle.Height);
                _centerGridOnFirstResize = false;
            }
        }

        /// <summary>
        /// 绘制栅格地板、网格线、机器人及视口信息。
        /// </summary>
        /// <param name="graphics">GDI+ 绘图表面。</param>
        /// <param name="viewArea">左侧视图绘制区域。</param>
        private void PaintScene(Graphics graphics, Rectangle viewArea)
        {
            graphics.SetClip(viewArea);
            graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.None;
            graphics.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;

            using (SolidBrush bgBrush = new SolidBrush(BackColor))
            {
                graphics.FillRectangle(bgBrush, viewArea);
            }

            float cellPixels = BaseCellPixels * _scale;
            float gridWidth = GridColumns * cellPixels;
            float gridHeight = GridRows * cellPixels;

            // 1. 绘制网格背景地板
            using (SolidBrush floorBrush = new SolidBrush(Color.White))
            {
                graphics.FillRectangle(floorBrush, _offsetX, _offsetY, gridWidth, gridHeight);
            }

            // 2. 绘制 30x30 栅格线与边框
            using (Pen gridPen = new Pen(Color.FromArgb(220, 225, 230), 1f))
            using (Pen borderPen = new Pen(Color.FromArgb(50, 62, 72), 2f))
            {
                for (int column = 0; column <= GridColumns; column++)
                {
                    float x = _offsetX + column * cellPixels;
                    graphics.DrawLine(gridPen, x, _offsetY, x, _offsetY + gridHeight);
                }

                for (int row = 0; row <= GridRows; row++)
                {
                    float y = _offsetY + row * cellPixels;
                    graphics.DrawLine(gridPen, _offsetX, y, _offsetX + gridWidth, y);
                }

                graphics.DrawRectangle(borderPen, _offsetX, _offsetY, gridWidth, gridHeight);
            }

            // 3. 高亮显示机器人当前/目标网格中心辅助点
            float robotCenterX = _offsetX + (float)((_robot.GridX + 0.5f) * cellPixels);
            float robotCenterY = _offsetY + (float)((_robot.GridY + 0.5f) * cellPixels);
            using (SolidBrush targetCellBrush = new SolidBrush(Color.FromArgb(30, 0, 120, 215)))
            {
                float cellX = _offsetX + _robot.GridX * cellPixels;
                float cellY = _offsetY + _robot.GridY * cellPixels;
                graphics.FillRectangle(targetCellBrush, cellX, cellY, cellPixels, cellPixels);
            }

            // 4. 绘制机器人
            _robot.Render(graphics, _offsetX, _offsetY, _scale, BaseCellPixels, CellSizeMeters);

            // 5. 绘制左上角简易 HUD 提示
            DrawViewportHud(graphics);

            graphics.ResetClip();
        }

        /// <summary>
        /// 绘制左上角的视口操作与单格物理尺寸说明。
        /// </summary>
        /// <param name="graphics">GDI+ 绘图表面。</param>
        private void DrawViewportHud(Graphics graphics)
        {
            using (SolidBrush panelBrush = new SolidBrush(Color.FromArgb(225, 255, 255, 255)))
            using (Pen panelPen = new Pen(Color.FromArgb(190, 200, 210)))
            using (SolidBrush textBrush = new SolidBrush(Color.FromArgb(40, 50, 60)))
            using (Font hudFont = new Font("Microsoft YaHei", 9f))
            {
                RectangleF hudRect = new RectangleF(12f, 12f, 260f, 68f);
                graphics.FillRectangle(panelBrush, hudRect);
                graphics.DrawRectangle(panelPen, Rectangle.Round(hudRect));

                string text =
                    "30 x 30 仓储栅格地图\r\n" +
                    "单格尺寸: 0.55m | 机器人居中对齐\r\n" +
                    "缩放: " + (_scale * 100f).ToString("0") + "% (滚轮缩放，左键拖拽平移)";

                graphics.DrawString(text, hudFont, textBrush, new RectangleF(18f, 16f, 245f, 58f));
            }
        }

        /// <summary>
        /// 在指定视图宽度和高度内将网格居中对齐。
        /// </summary>
        /// <param name="viewWidth">视图可用宽度。</param>
        /// <param name="viewHeight">视图可用高度。</param>
        private void CenterGrid(int viewWidth, int viewHeight)
        {
            float gridSize = BaseCellPixels * GridColumns * _scale;
            _offsetX = (viewWidth - gridSize) * 0.5f;
            _offsetY = (viewHeight - gridSize) * 0.5f;
            Invalidate();
        }

        /// <summary>
        /// 鼠标按下时判定是否在地图视口区域并发起平移。
        /// </summary>
        /// <param name="sender">事件源。</param>
        /// <param name="e">鼠标参数。</param>
        private void OnMouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left || e.X >= ClientRectangle.Width - _controlPanel.Width)
            {
                return;
            }

            _isPanning = true;
            _lastMousePosition = e.Location;
            Cursor = Cursors.SizeAll;
            Capture = true;
        }

        /// <summary>
        /// 鼠标拖动更新视口偏移量。
        /// </summary>
        /// <param name="sender">事件源。</param>
        /// <param name="e">鼠标参数。</param>
        private void OnMouseMove(object sender, MouseEventArgs e)
        {
            if (!_isPanning)
            {
                return;
            }

            _offsetX += e.X - _lastMousePosition.X;
            _offsetY += e.Y - _lastMousePosition.Y;
            _lastMousePosition = e.Location;
            Invalidate();
        }

        /// <summary>
        /// 鼠标释放结束视口平移。
        /// </summary>
        /// <param name="sender">事件源。</param>
        /// <param name="e">鼠标参数。</param>
        private void OnMouseUp(object sender, MouseEventArgs e)
        {
            if (e.Button != MouseButtons.Left)
            {
                return;
            }

            _isPanning = false;
            Cursor = Cursors.Default;
            Capture = false;
        }

        /// <summary>
        /// 鼠标滚轮在地图区域缩放视口。
        /// </summary>
        /// <param name="sender">事件源。</param>
        /// <param name="e">鼠标参数。</param>
        private void OnMouseWheel(object sender, MouseEventArgs e)
        {
            if (e.X >= ClientRectangle.Width - _controlPanel.Width)
            {
                return;
            }

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
            Invalidate();
        }
    }
}