using System;
using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;
using SkiaSharp;
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
        private bool _centerGridOnFirstResize = true;

        private readonly Robot _robot;
        private readonly Timer _simulationTimer;
        private readonly Stopwatch _stopwatch;
        private double _lastElapsedSeconds;

        private SkiaCanvasControl _skiaCanvas;
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

        public MainForm()
        {
            // 初始化主窗体、Skia 画布、机器人模型与控制面板
            Text = "仓储物流多智能体调度仿真 (SkiaSharp渲染)";
            Size = new Size(1180, 800);
            StartPosition = FormStartPosition.CenterScreen;
            MinimumSize = new Size(800, 600);
            DoubleBuffered = true;
            BackColor = Color.FromArgb(245, 247, 250);
            KeyPreview = true;

            _robot = new Robot(14, 14, CellSizeMeters, RobotOrientation.Up);

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
            // 创建并布局 Skia 画布与右侧参数调节面板
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
            lblTitle.Text = "机器人参数调节 (SkiaSharp)";
            lblTitle.Font = new Font("Microsoft YaHei", 10.5f, FontStyle.Bold);
            lblTitle.ForeColor = Color.FromArgb(30, 45, 60);
            lblTitle.Location = new Point(14, currentY);
            lblTitle.Size = new Size(255, 24);
            _controlPanel.Controls.Add(lblTitle);
            currentY += 36;

            _controlPanel.Controls.Add(CreateHorizontalSeparator(currentY));
            currentY += 16;

            // 1. 最大速度调节
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
            currentY += 56;

            // 2. 加速度调节
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
            currentY += 56;

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

            _controlPanel.Controls.Add(CreateHorizontalSeparator(currentY));
            currentY += 16;

            // 3. 动作控制指令
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
            currentY += 42;

            _controlPanel.Controls.Add(CreateHorizontalSeparator(currentY));
            currentY += 16;

            // 4. 实时状态监测
            Label lblMonitorTitle = new Label();
            lblMonitorTitle.Text = "实时状态监测 (SkiaSharp):";
            lblMonitorTitle.Font = new Font("Microsoft YaHei", 9f, FontStyle.Bold);
            lblMonitorTitle.Location = new Point(14, currentY);
            lblMonitorTitle.Size = new Size(250, 24);
            _controlPanel.Controls.Add(lblMonitorTitle);
            currentY += 28;

            _lblStatusInfo = new Label();
            _lblStatusInfo.Location = new Point(14, currentY);
            _lblStatusInfo.Size = new Size(250, 160);
            _lblStatusInfo.Font = new Font("Consolas", 9f);
            _lblStatusInfo.BackColor = Color.FromArgb(246, 248, 250);
            _lblStatusInfo.BorderStyle = BorderStyle.FixedSingle;
            _lblStatusInfo.Padding = new Padding(6);
            _controlPanel.Controls.Add(_lblStatusInfo);
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
            // 将机器人复位回到网格中心点 (14, 14)
            _robot.ResetToGrid(14, 14, CellSizeMeters);
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
            // 驱动仿真时间步长并触发 Skia 画布重绘与状态面板刷新
            double currentSeconds = _stopwatch.Elapsed.TotalSeconds;
            float deltaTime = (float)(currentSeconds - _lastElapsedSeconds);
            _lastElapsedSeconds = currentSeconds;

            if (deltaTime > 0.1f)
            {
                deltaTime = 0.1f;
            }

            _robot.Update(deltaTime, CellSizeMeters);
            UpdateStatusLabel();
            _skiaCanvas.Invalidate();
        }

        private void UpdateStatusLabel()
        {
            // 刷新右侧状态栏的网格坐标、物理坐标、角度与瞬时速度
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
                "渲染引擎: SkiaSharp 2.88\r\n" +
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

        protected override void OnResize(EventArgs e)
        {
            // 窗口首次加载完成时居中网格地图
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
            // 使用 SkiaSharp 绘制 30x30 离散栅格、机器人与 HUD 说明
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
                Color = new SKColor(0, 120, 215, 40),
                Style = SKPaintStyle.Fill
            })
            {
                float cellX = _offsetX + _robot.GridX * cellPixels;
                float cellY = _offsetY + _robot.GridY * cellPixels;
                canvas.DrawRect(cellX, cellY, cellPixels, cellPixels, highlightPaint);
            }

            // 4. 绘制机器人
            _robot.Render(canvas, _offsetX, _offsetY, _scale, BaseCellPixels, CellSizeMeters);

            // 5. 绘制视口 HUD 提示面板
            DrawViewportHud(canvas);
        }

        private void DrawViewportHud(SKCanvas canvas)
        {
            // 在左上角使用 SkiaSharp 绘制圆角 HUD 视口参数说明卡片
            SKRect hudRect = new SKRect(12f, 12f, 280f, 82f);

            using (var panelPaint = new SKPaint
            {
                Color = new SKColor(255, 255, 255, 230),
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
                canvas.DrawText("缩放: " + (_scale * 100f).ToString("0") + "% (滚轮缩放，左键拖拽平移)", 20f, 72f, textPaint);
            }
        }

        private void OnCanvasMouseDown(object sender, MouseEventArgs e)
        {
            // 响应鼠标左键按下并发起地图视口平移
            // e: 鼠标事件数据
            if (e.Button != MouseButtons.Left)
            {
                return;
            }

            _isPanning = true;
            _lastMousePosition = e.Location;
            _skiaCanvas.Cursor = Cursors.SizeAll;
            _skiaCanvas.Capture = true;
        }

        private void OnCanvasMouseMove(object sender, MouseEventArgs e)
        {
            // 响应鼠标拖拽并更新视口偏移像素
            if (!_isPanning)
            {
                return;
            }

            _offsetX += e.X - _lastMousePosition.X;
            _offsetY += e.Y - _lastMousePosition.Y;
            _lastMousePosition = e.Location;
            _skiaCanvas.Invalidate();
        }

        private void OnCanvasMouseUp(object sender, MouseEventArgs e)
        {
            // 响应鼠标按键释放并停止平移操作
            if (e.Button != MouseButtons.Left)
            {
                return;
            }

            _isPanning = false;
            _skiaCanvas.Cursor = Cursors.Default;
            _skiaCanvas.Capture = false;
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
