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
    public sealed partial class MainForm : Form
    {
        private bool _firstLayout = true;

        private readonly SimulationApp _app;
        private readonly SkiaWarehouseRenderer _renderer;
        private readonly SkiaUiManager _uiManager;
        private readonly ViewportController _viewport;
        private SingleRobotPresenter _singleRobotPresenter;
        private MultiRobotPresenter _multiRobotPresenter;

        private SkiaCanvasControl _skiaCanvas;
        private readonly Timer _renderTimer;
        private readonly Stopwatch _stopwatch;
        private double _lastElapsedSeconds;
        private double _lastPaintSeconds = -1.0;
        private double _averageFrameSeconds;

        // 手动控制长按状态与受控机器人选择 (按钮与键盘按键完全同步，支持单机与多机点选)
        private int? _selectedRobotId = 1;
        private bool _isManualForwardActive = false;
        private bool _isManualTurnLeftActive = false;
        private bool _isManualTurnRightActive = false;

        private bool _chkShowPathChecked = true;
        private bool _chkShowTargetLinesChecked = true;
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
            _viewport = new ViewportController();

            Text = "仓储多机器人 RCS 调度仿真系统 (SkiaSharp 矢量渲染)";
            Size = new Size(1260, 840);
            StartPosition = FormStartPosition.CenterScreen;
            MinimumSize = new Size(960, 640);
            DoubleBuffered = true;
            BackColor = Color.FromArgb(240, 244, 248);
            KeyPreview = true;

            InitializeControls();
            if (_cmbMultiAlgorithm != null)
            {
                _cmbMultiAlgorithm.DisplayMember = "Name";
                _cmbMultiAlgorithm.ValueMember = "Id";
                _cmbMultiAlgorithm.DataSource = MultiAgentPlannerRegistry.GetAll().ToList();
            }
            BuildSkiaButtons();

            if (_app?.Engine != null)
            {
                _app.Engine.Config.EnableBackgroundWorker = true;
            }

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
                _app?.Dispose();
            }
            base.Dispose(disposing);
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
                    _multiRobotPresenter?.GenerateRandomRobots(defaultCount);
                }
                _selectedRobotId = null;
            }
            else
            {
                _selectedRobotId = 1;
                _singleRobotPresenter?.ResetTarget();
                _multiRobotPresenter?.ClearTargets();
                HandleResetPosition();
            }
            _skiaCanvas?.Invalidate();
        }

        private void OnMultiAlgorithmSelectedIndexChanged(object sender, EventArgs e)
        {
            // 响应多机调度算法切换并动态调整专属参数面板显隐。
            UpdateMultiAlgoParamsVisibility();
            _skiaCanvas?.Invalidate();
        }

        private void OnRobotCountSelectedIndexChanged(object sender, EventArgs e)
        {
            // 响应机器人机群数量选择变更。
            if (_cmbRobotCount.SelectedItem is int count)
            {
                _selectedRobotId = null;
                _multiRobotPresenter?.GenerateRandomRobots(count);
                _skiaCanvas?.Invalidate();
            }
        }

        private void HandleRandomizeRobots()
        {
            // 响应重新随机生成机群与对角目标点按钮点击。
            int count = 20;
            if (_cmbRobotCount?.SelectedItem is int selCount) count = selCount;
            _selectedRobotId = null;
            _multiRobotPresenter?.GenerateRandomRobots(count);
            _skiaCanvas?.Invalidate();
        }

        private void HandleStartMultiNavigation()
        {
            // 启动多机协同调度导航任务。
            var selectedPlanner = _cmbMultiAlgorithm?.SelectedItem as IMultiAgentPathPlanner
                ?? MultiAgentPlannerRegistry.Default;

            var options = new MultiAgentPlanOptions
            {
                WhcaWindowK = (int)(_numWhcaK?.Value ?? 18),
                EcbsSuboptimality = (double)(_numEcbsW?.Value ?? 1.30m),
                TimeoutMilliseconds = 4000
            };
            _multiRobotPresenter?.HandleStartMultiNavigation(this, selectedPlanner, options);
            _skiaCanvas?.Invalidate();
        }

        private void HandleStopMultiNavigation()
        {
            // 停止多机协同调度。
            _multiRobotPresenter?.HandleStopMultiNavigation();
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
            _singleRobotPresenter?.OnAlgorithmChanged();
            _skiaCanvas?.Invalidate();
        }

        public void SetSelectedTarget(Point target)
        {
            // 设置当前选定的目标网格并更新状态提示与路径规划。
            // target: 选中的离散网格坐标。
            _singleRobotPresenter?.OnGridClicked(target);
            _skiaCanvas?.Invalidate();
        }

        private void HandleStartNavigation()
        {
            // 启动单机自动寻路导航任务。
            _singleRobotPresenter?.HandleStartNavigation(this);
            _skiaCanvas?.Invalidate();
        }

        private void HandleCancelNavigation()
        {
            // 取消单机导航任务并平缓停止。
            _singleRobotPresenter?.HandleCancelNavigation();
            _skiaCanvas?.Invalidate();
        }

        private void SetManualDrive(ManualDriveAction action, bool active)
        {
            // 向当前选定的机器人统一分发手动长按或释放制动信号。
            // action: 手动驾驶动作类型; active: 控制信号是否处于按下激活状态。
            switch (action)
            {
                case ManualDriveAction.Forward: _isManualForwardActive = active; break;
                case ManualDriveAction.TurnLeft: _isManualTurnLeftActive = active; break;
                case ManualDriveAction.TurnRight: _isManualTurnRightActive = active; break;
            }

            int targetId = _selectedRobotId ?? _app.Engine?.Robots?.FirstOrDefault()?.Id ?? 1;
            _app.ManualDriveRobot(targetId, action, active);
            _skiaCanvas?.Invalidate();
        }

        private void SetManualForward(bool active) => SetManualDrive(ManualDriveAction.Forward, active);
        private void SetManualTurnLeft(bool active) => SetManualDrive(ManualDriveAction.TurnLeft, active);
        private void SetManualTurnRight(bool active) => SetManualDrive(ManualDriveAction.TurnRight, active);

        private void HandleResetPosition()
        {
            // 将机器人复位到地图默认初始位姿。
            var r = _app.Engine?.Robots?.FirstOrDefault();
            if (r == null) return;

            var mc = _app.Engine.GetMotionController(r.Id);
            mc?.ClearPath();

            int initX = _app.CurrentMapData?.InitialRobots?.FirstOrDefault()?.X ?? 15;
            int initY = _app.CurrentMapData?.InitialRobots?.FirstOrDefault()?.Y ?? 27;
            float centerCoordX = (initX + 0.5f) * _app.CurrentMap.CellSizeMeters;
            float centerCoordY = (initY + 0.5f) * _app.CurrentMap.CellSizeMeters;
            r.TeleportForReset(new Pose2D(centerCoordX, centerCoordY, -(float)Math.PI * 0.5f));
            _app.Engine.PublishSnapshot();

            _singleRobotPresenter?.ResetTarget();
            if (_lblTargetInfo != null)
            {
                _lblTargetInfo.Text = $"已复位到初始位 [{initX}, {initY}]";
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
            _viewport.CenterGridInView(ClientSize.Width, ClientSize.Height, _app.CurrentMap.Width, _app.CurrentMap.Height);
        }

        private void OnRenderTimerTick(object sender, EventArgs e)
        {
            // 响应固定帧率渲染定时器并刷新 Skia 画布呈现。
            // sender: 触发事件的对象源; e: 定时器事件参数。
            double now = _stopwatch.Elapsed.TotalSeconds;
            _lastElapsedSeconds = now;

            // 长按手动驾驶信号持续维持推进 (通过线程安全并发命令队列异步分发)
            int activeId = _selectedRobotId ?? _app.Engine?.Robots?.FirstOrDefault()?.Id ?? 1;
            if (_isManualForwardActive) _app.MaintainManualDrive(activeId, ManualDriveAction.Forward);
            else if (_isManualTurnLeftActive) _app.MaintainManualDrive(activeId, ManualDriveAction.TurnLeft);
            else if (_isManualTurnRightActive) _app.MaintainManualDrive(activeId, ManualDriveAction.TurnRight);

            // 状态机与界面控件可用性同步
            if (_currentMode == SimulationMode.SingleRobot)
            {
                _singleRobotPresenter?.UpdateTickState();
            }
            else if (_currentMode == SimulationMode.MultiRobot)
            {
                _multiRobotPresenter?.UpdateTickState();
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

            float interpolationAlpha = 1f;
            if (snapshot != null && snapshot.IsRunning && !snapshot.IsPaused)
            {
                double currentWallClock = (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
                double timeSinceSnapshot = Math.Max(0.0, currentWallClock - snapshot.WallClockTimeSeconds);
                float fixedDt = _app.Engine?.Config?.FixedDeltaTime ?? 0.1f;
                if (fixedDt > 0f)
                {
                    interpolationAlpha = (float)Math.Clamp(timeSinceSnapshot / fixedDt, 0.0, 1.0);
                }
            }

            double multiElapsedTime = _multiRobotPresenter?.NavElapsedTime ?? 0.0;
            int completedTasksCount = _multiRobotPresenter?.GetCompletedTasksCount() ?? 0;
            double taskEfficiency = _multiRobotPresenter?.TaskEfficiencyPerMinute ?? 0.0;

            _renderer.Render(
                canvas,
                width,
                height,
                _app.CurrentMap,
                snapshot,
                _viewport.OffsetX,
                _viewport.OffsetY,
                _viewport.Scale,
                ViewportController.BaseCellPixels,
                _currentMode,
                _singleRobotPresenter?.SelectedTarget,
                _chkShowPathChecked,
                _chkShowTargetLinesChecked,
                _multiRobotPresenter?.MultiTargets,
                _viewport.IsViewportHudMinimized,
                _viewport.IsStatusHudMinimized,
                ref _viewport.ViewportHudRect,
                ref _viewport.StatusHudRect,
                ref _viewport.ViewportMinBtnRect,
                ref _viewport.StatusMinBtnRect,
                _renderFps,
                interpolationAlpha,
                _selectedRobotId,
                multiElapsedTime,
                completedTasksCount,
                taskEfficiency);
        }

        // ==================== 鼠标事件转发与命中测试 ====================

        private void OnCanvasMouseDown(object sender, MouseEventArgs e)
        {
            // 响应鼠标按下事件并执行 HUD 折叠按钮与 UI 命中测试或启动视口平移。
            // e: 鼠标事件参数。
            _skiaCanvas?.Focus();

            if (_viewport.HandleMouseDown(e.Location, e.Button, out bool hudMinBtnClicked))
            {
                if (hudMinBtnClicked)
                {
                    _skiaCanvas?.Invalidate();
                }
                return;
            }

            if (_uiManager.OnMouseDown(e.X, e.Y))
            {
                _skiaCanvas?.Invalidate();
            }
        }

        private void OnCanvasMouseMove(object sender, MouseEventArgs e)
        {
            // 响应鼠标移动事件并更新按钮悬停状态或视口平移偏移。
            bool uiHandled = _uiManager.OnMouseMove(e.X, e.Y);
            bool panHandled = _viewport.HandleMouseMove(e.Location);

            if (panHandled || uiHandled)
            {
                _skiaCanvas?.Invalidate();
            }
        }

        private void OnCanvasMouseUp(object sender, MouseEventArgs e)
        {
            // 响应鼠标抬起事件并触发按钮点击、网格点选或结束视口平移。
            bool panEnded = _viewport.HandleMouseUp(e.Location, out bool isClick);

            if (_uiManager.OnMouseUp(e.X, e.Y))
            {
                _skiaCanvas?.Invalidate();
                return;
            }

            if (panEnded)
            {
                _skiaCanvas?.Invalidate();
            }

            // 单击未发生拖拽时进行机器人点选接管或目标网格拾取
            if (isClick && _app.CurrentMap != null)
            {
                if (_viewport.ScreenToGrid(e.Location, _app.CurrentMap.Width, _app.CurrentMap.Height, out Point clickedPoint))
                {
                    var clickedGrid = new GridPoint(clickedPoint.X, clickedPoint.Y);

                    // 优先检查是否点选中了某台机器人 (支持栅格确认位置或车体几何范围匹配)
                    float cellMeters = _app.CurrentMap.CellSizeMeters;
                    var hitRobot = _app.Engine?.Robots?.FirstOrDefault(r =>
                    {
                        if (r.ConfirmedGrid == clickedGrid) return true;
                        float dx = r.Position.X - (clickedGrid.X + 0.5f) * cellMeters;
                        float dy = r.Position.Y - (clickedGrid.Y + 0.5f) * cellMeters;
                        return (dx * dx + dy * dy) <= (cellMeters * cellMeters * 0.36f);
                    });

                    if (hitRobot != null)
                    {
                        // 多机模式下若已选中该机器人，再次点击则取消选中并切回机群整体视角
                        if (_currentMode == SimulationMode.MultiRobot && _selectedRobotId == hitRobot.Id)
                        {
                            _selectedRobotId = null;
                        }
                        else
                        {
                            _selectedRobotId = hitRobot.Id;
                        }
                        _skiaCanvas?.Invalidate();
                    }
                    else if (_currentMode == SimulationMode.SingleRobot)
                    {
                        if (!_app.CurrentMap.IsObstacle(clickedGrid))
                        {
                            _singleRobotPresenter?.OnGridClicked(clickedPoint);
                            _skiaCanvas?.Invalidate();
                        }
                    }
                    else if (_currentMode == SimulationMode.MultiRobot)
                    {
                        // 多机模式下点击空白栅格，取消单车选中，切回机群整体状态监测
                        if (_selectedRobotId != null)
                        {
                            _selectedRobotId = null;
                            _skiaCanvas?.Invalidate();
                        }
                    }
                }
            }
        }

        private void OnCanvasMouseWheel(object sender, MouseEventArgs e)
        {
            // 响应鼠标滚轮滚动并以光标位置为锚点执行平滑缩放。
            if (_viewport.HandleMouseWheel(e.Location, e.Delta))
            {
                _skiaCanvas?.Invalidate();
            }
        }

        // ==================== 快捷键与连续按键支持 ====================

        protected override void OnKeyDown(KeyEventArgs e)
        {
            // 响应键盘按下事件并转换为手动控制长按状态。
            // e: 键盘事件参数。
            base.OnKeyDown(e);

            switch (e.KeyCode)
            {
                case Keys.W:
                case Keys.Up:
                    SetManualForward(true);
                    break;
                case Keys.A:
                case Keys.Left:
                    SetManualTurnLeft(true);
                    break;
                case Keys.D:
                case Keys.Right:
                    SetManualTurnRight(true);
                    break;
                case Keys.Space:
                    if (_currentMode == SimulationMode.SingleRobot)
                    {
                        HandleResetPosition();
                    }
                    break;
            }
        }

        protected override void OnKeyUp(KeyEventArgs e)
        {
            // 响应键盘松开事件并释放手动控制状态执行减速停止。
            base.OnKeyUp(e);

            switch (e.KeyCode)
            {
                case Keys.W:
                case Keys.Up:
                    SetManualForward(false);
                    break;
                case Keys.A:
                case Keys.Left:
                    SetManualTurnLeft(false);
                    break;
                case Keys.D:
                case Keys.Right:
                    SetManualTurnRight(false);
                    break;
            }
        }

        protected override void OnDeactivate(EventArgs e)
        {
            // 窗体失去焦点时自动释放全部手动控制状态以防动作卡滞。
            base.OnDeactivate(e);
            SetManualForward(false);
            SetManualTurnLeft(false);
            SetManualTurnRight(false);
        }

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            // 拦截并处理全局仿真控制快捷键。
            // msg: Windows 消息对象; keyData: 触发的按键键码数据。
            switch (keyData)
            {
                case Keys.S:
                    _app.Step();
                    return true;
                case Keys.R:
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
