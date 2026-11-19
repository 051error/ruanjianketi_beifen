using System;
using System.Drawing;
using System.Drawing.Drawing2D;

namespace WarehouseGridSimulation.Models
{
    /// <summary>
    /// 表示机器人在离散栅格中的四正交朝向。
    /// </summary>
    public enum RobotOrientation
    {
        /// <summary>
        /// 朝上（-Y 方向，270 度）。
        /// </summary>
        Up = 0,

        /// <summary>
        /// 朝右（+X 方向，0 度）。
        /// </summary>
        Right = 1,

        /// <summary>
        /// 朝下（+Y 方向，90 度）。
        /// </summary>
        Down = 2,

        /// <summary>
        /// 朝左（-X 方向，180 度）。
        /// </summary>
        Left = 3
    }

    /// <summary>
    /// 表示机器人当前的动作状态。
    /// </summary>
    public enum RobotState
    {
        /// <summary>
        /// 静止于网格中心，等待指令。
        /// </summary>
        Idle = 0,

        /// <summary>
        /// 正在从当前网格平滑移动至相邻网格。
        /// </summary>
        Moving = 1,

        /// <summary>
        /// 正在进行 90 度原地旋转。
        /// </summary>
        Turning = 2
    }

    /// <summary>
    /// 表示离散仓储栅格中运动的单个智能体/机器人（AGV）。
    /// </summary>
    public class Robot
    {
        /// <summary>
        /// 机器人物理底盘长度（米，小于单格尺寸 0.55m）。
        /// </summary>
        public const float ChassisLengthMeters = 0.42f;

        /// <summary>
        /// 机器人物理底盘宽度（米，小于单格尺寸 0.55m）。
        /// </summary>
        public const float ChassisWidthMeters = 0.38f;

        /// <summary>
        /// 获取或设置机器人当前所处的离散网格列索引（0 到 29）。
        /// </summary>
        public int GridX { get; set; }

        /// <summary>
        /// 获取或设置机器人当前所处的离散网格行索引（0 到 29）。
        /// </summary>
        public int GridY { get; set; }

        /// <summary>
        /// 获取或设置目标网格列索引。
        /// </summary>
        public int TargetGridX { get; set; }

        /// <summary>
        /// 获取或设置目标网格行索引。
        /// </summary>
        public int TargetGridY { get; set; }

        /// <summary>
        /// 获取或设置机器人的正交朝向。
        /// </summary>
        public RobotOrientation Orientation { get; set; }

        /// <summary>
        /// 获取机器人的当前动作状态。
        /// </summary>
        public RobotState State { get; private set; }

        /// <summary>
        /// 获取机器人当前的实时物理 X 坐标（米）。
        /// </summary>
        public float PositionX { get; private set; }

        /// <summary>
        /// 获取机器人当前的实时物理 Y 坐标（米）。
        /// </summary>
        public float PositionY { get; private set; }

        /// <summary>
        /// 获取机器人当前的实时渲染朝向角度（度）。
        /// </summary>
        public float CurrentAngle { get; private set; }

        /// <summary>
        /// 获取或设置最大线速度（米/秒），默认为 1.5 米/秒，GUI 可调。
        /// </summary>
        public float MaxSpeed { get; set; }

        /// <summary>
        /// 获取或设置线加速度（米/秒²），GUI 可调。
        /// </summary>
        public float Acceleration { get; set; }

        /// <summary>
        /// 获取当前的瞬时线速度（米/秒）。
        /// </summary>
        public float CurrentSpeed { get; private set; }

        // 运动物理计算内部变量
        private float _startPhysX;
        private float _startPhysY;
        private float _targetPhysX;
        private float _targetPhysY;

        private float _startAngle;
        private float _targetAngle;
        private float _turnElapsed;
        private float _turnDuration;

        private float _moveElapsed;
        private float _totalMoveTime;
        private float _accelTime;
        private float _constTime;
        private float _peakSpeed;
        private float _moveDistanceMeters;

        /// <summary>
        /// 初始化机器人实例，放置在指定网格中心并对齐正交方向。
        /// </summary>
        /// <param name="gridX">初始网格列坐标（0 ~ 29）。</param>
        /// <param name="gridY">初始网格行坐标（0 ~ 29）。</param>
        /// <param name="cellSizeMeters">单网格物理尺寸（米）。</param>
        /// <param name="orientation">初始正交朝向。</param>
        public Robot(int gridX, int gridY, float cellSizeMeters, RobotOrientation orientation)
        {
            GridX = gridX;
            GridY = gridY;
            TargetGridX = gridX;
            TargetGridY = gridY;
            Orientation = orientation;
            State = RobotState.Idle;

            MaxSpeed = 1.5f;
            Acceleration = 2.0f;
            CurrentSpeed = 0f;

            CurrentAngle = GetBaseAngle(orientation);
            _targetAngle = CurrentAngle;

            PositionX = (gridX + 0.5f) * cellSizeMeters;
            PositionY = (gridY + 0.5f) * cellSizeMeters;
            _startPhysX = PositionX;
            _startPhysY = PositionY;
            _targetPhysX = PositionX;
            _targetPhysY = PositionY;
        }

        /// <summary>
        /// 将正交朝向枚举转换为标准的角度值（0° 右, 90° 下, 180° 左, 270° 上）。
        /// </summary>
        /// <param name="orientation">正交朝向枚举。</param>
        /// <returns>对应的度数。</returns>
        public static float GetBaseAngle(RobotOrientation orientation)
        {
            switch (orientation)
            {
                case RobotOrientation.Right:
                    return 0f;
                case RobotOrientation.Down:
                    return 90f;
                case RobotOrientation.Left:
                    return 180f;
                case RobotOrientation.Up:
                    return 270f;
                default:
                    return 0f;
            }
        }

        /// <summary>
        /// 请求向机器人当前正交朝向前进一格。
        /// </summary>
        /// <param name="gridColumns">网格总列数（例如 30）。</param>
        /// <param name="gridRows">网格总行数（例如 30）。</param>
        /// <param name="cellSizeMeters">单网格物理尺寸（米）。</param>
        /// <returns>若成功发起前进动作返回 true；若遇边界或正处于动作中则返回 false。</returns>
        public bool TryMoveForward(int gridColumns, int gridRows, float cellSizeMeters)
        {
            if (State != RobotState.Idle)
            {
                return false;
            }

            int nextX = GridX;
            int nextY = GridY;

            switch (Orientation)
            {
                case RobotOrientation.Up:
                    nextY -= 1;
                    break;
                case RobotOrientation.Right:
                    nextX += 1;
                    break;
                case RobotOrientation.Down:
                    nextY += 1;
                    break;
                case RobotOrientation.Left:
                    nextX -= 1;
                    break;
            }

            // 越界检查，确保在 30x30 栅格内
            if (nextX < 0 || nextX >= gridColumns || nextY < 0 || nextY >= gridRows)
            {
                return false;
            }

            TargetGridX = nextX;
            TargetGridY = nextY;

            _startPhysX = (GridX + 0.5f) * cellSizeMeters;
            _startPhysY = (GridY + 0.5f) * cellSizeMeters;
            _targetPhysX = (TargetGridX + 0.5f) * cellSizeMeters;
            _targetPhysY = (TargetGridY + 0.5f) * cellSizeMeters;

            _moveDistanceMeters = cellSizeMeters;
            _moveElapsed = 0f;

            // 梯形或三角形速度规划（对称加减速）：
            // 达到最大速度所需的加速位移 d_acc = v^2 / (2 * a)
            float safeAccel = Math.Max(0.1f, Acceleration);
            float safeMaxSpeed = Math.Max(0.1f, MaxSpeed);

            float accelDist = (safeMaxSpeed * safeMaxSpeed) / (2.0f * safeAccel);
            if (accelDist * 2.0f >= _moveDistanceMeters)
            {
                // 三角形加减速曲线：未达到最大速度就必须开始减速
                _peakSpeed = (float)Math.Sqrt(safeAccel * _moveDistanceMeters);
                _accelTime = _peakSpeed / safeAccel;
                _constTime = 0f;
                _totalMoveTime = _accelTime * 2.0f;
            }
            else
            {
                // 梯形曲线：加速 -> 匀速 -> 减速
                _peakSpeed = safeMaxSpeed;
                _accelTime = _peakSpeed / safeAccel;
                float constDist = _moveDistanceMeters - accelDist * 2.0f;
                _constTime = constDist / _peakSpeed;
                _totalMoveTime = _accelTime * 2.0f + _constTime;
            }

            State = RobotState.Moving;
            return true;
        }

        /// <summary>
        /// 请求机器人逆时针旋转 90 度（左转）。
        /// </summary>
        /// <returns>若成功发起旋转返回 true，若正处于动作中则返回 false。</returns>
        public bool TryTurnLeft()
        {
            if (State != RobotState.Idle)
            {
                return false;
            }

            RobotOrientation newOrientation = (RobotOrientation)(((int)Orientation - 1 + 4) % 4);
            StartTurning(newOrientation, -90f);
            return true;
        }

        /// <summary>
        /// 请求机器人顺时针旋转 90 度（右转）。
        /// </summary>
        /// <returns>若成功发起旋转返回 true，若正处于动作中则返回 false。</returns>
        public bool TryTurnRight()
        {
            if (State != RobotState.Idle)
            {
                return false;
            }

            RobotOrientation newOrientation = (RobotOrientation)(((int)Orientation + 1) % 4);
            StartTurning(newOrientation, 90f);
            return true;
        }

        /// <summary>
        /// 启动 90 度平滑旋转过程。
        /// </summary>
        /// <param name="newOrientation">旋转完成后的目标正交朝向。</param>
        /// <param name="deltaAngle">旋转角度变化量（-90 为左转，+90 为右转）。</param>
        private void StartTurning(RobotOrientation newOrientation, float deltaAngle)
        {
            Orientation = newOrientation;
            _startAngle = CurrentAngle;
            _targetAngle = CurrentAngle + deltaAngle;
            _turnElapsed = 0f;
            // 90度旋转耗时约 0.25 秒，平滑自然
            _turnDuration = 0.25f;
            State = RobotState.Turning;
        }

        /// <summary>
        /// 重置机器人回到指定网格中心并处于空闲静止状态。
        /// </summary>
        /// <param name="gridX">网格列坐标。</param>
        /// <param name="gridY">网格行坐标。</param>
        /// <param name="cellSizeMeters">单网格物理尺寸（米）。</param>
        public void ResetToGrid(int gridX, int gridY, float cellSizeMeters)
        {
            GridX = gridX;
            GridY = gridY;
            TargetGridX = gridX;
            TargetGridY = gridY;
            Orientation = RobotOrientation.Up;
            State = RobotState.Idle;

            CurrentSpeed = 0f;
            CurrentAngle = GetBaseAngle(Orientation);
            _targetAngle = CurrentAngle;

            PositionX = (gridX + 0.5f) * cellSizeMeters;
            PositionY = (gridY + 0.5f) * cellSizeMeters;
            _startPhysX = PositionX;
            _startPhysY = PositionY;
            _targetPhysX = PositionX;
            _targetPhysY = PositionY;
        }

        /// <summary>
        /// 根据每帧时间步长更新机器人的插值位置与朝向。
        /// </summary>
        /// <param name="deltaTime">经过的时间步长（秒）。</param>
        /// <param name="cellSizeMeters">单网格物理尺寸（米）。</param>
        public void Update(float deltaTime, float cellSizeMeters)
        {
            if (deltaTime <= 0f)
            {
                return;
            }

            if (State == RobotState.Turning)
            {
                _turnElapsed += deltaTime;
                if (_turnElapsed >= _turnDuration)
                {
                    // 转向结束，精准锁定目标角度，并规格化到 [0, 360)
                    CurrentAngle = GetBaseAngle(Orientation);
                    _targetAngle = CurrentAngle;
                    State = RobotState.Idle;
                }
                else
                {
                    float progress = _turnElapsed / _turnDuration;
                    // 使用平滑的三次缓动函数 S 曲线
                    float smooth = progress * progress * (3.0f - 2.0f * progress);
                    CurrentAngle = _startAngle + (_targetAngle - _startAngle) * smooth;
                }
            }
            else if (State == RobotState.Moving)
            {
                _moveElapsed += deltaTime;
                if (_moveElapsed >= _totalMoveTime)
                {
                    // 移动结束，精准对齐目标网格中心
                    GridX = TargetGridX;
                    GridY = TargetGridY;
                    PositionX = (GridX + 0.5f) * cellSizeMeters;
                    PositionY = (GridY + 0.5f) * cellSizeMeters;
                    CurrentSpeed = 0f;
                    State = RobotState.Idle;
                }
                else
                {
                    // 根据梯形/三角形曲线计算当前经过的位移和当前瞬时速度
                    float s = 0f;
                    float safeAccel = Math.Max(0.1f, Acceleration);

                    if (_moveElapsed < _accelTime)
                    {
                        // 1. 加速段
                        s = 0.5f * safeAccel * _moveElapsed * _moveElapsed;
                        CurrentSpeed = safeAccel * _moveElapsed;
                    }
                    else if (_moveElapsed < _accelTime + _constTime)
                    {
                        // 2. 匀速段
                        float accelDist = 0.5f * safeAccel * _accelTime * _accelTime;
                        float constElapsed = _moveElapsed - _accelTime;
                        s = accelDist + _peakSpeed * constElapsed;
                        CurrentSpeed = _peakSpeed;
                    }
                    else
                    {
                        // 3. 减速段
                        float decelElapsed = _moveElapsed - (_accelTime + _constTime);
                        float remainingTime = _accelTime - decelElapsed;
                        if (remainingTime < 0f) remainingTime = 0f;

                        float accelDist = 0.5f * safeAccel * _accelTime * _accelTime;
                        float constDist = _peakSpeed * _constTime;
                        float decelDistCovered = (0.5f * safeAccel * _accelTime * _accelTime) - (0.5f * safeAccel * remainingTime * remainingTime);

                        s = accelDist + constDist + decelDistCovered;
                        CurrentSpeed = safeAccel * remainingTime;
                    }

                    float ratio = (_moveDistanceMeters > 0.0001f) ? (s / _moveDistanceMeters) : 1f;
                    if (ratio > 1f) ratio = 1f;

                    PositionX = _startPhysX + (_targetPhysX - _startPhysX) * ratio;
                    PositionY = _startPhysY + (_targetPhysY - _startPhysY) * ratio;
                }
            }
            else
            {
                CurrentSpeed = 0f;
                // 静止时确保处在网格绝对中心
                PositionX = (GridX + 0.5f) * cellSizeMeters;
                PositionY = (GridY + 0.5f) * cellSizeMeters;
            }
        }

        /// <summary>
        /// 使用 GDI+ 绘制机器人外形与朝向标志。
        /// </summary>
        /// <param name="graphics">GDI+ 绘图表面。</param>
        /// <param name="offsetX">视口当前 X 轴像素偏移量。</param>
        /// <param name="offsetY">视口当前 Y 轴像素偏移量。</param>
        /// <param name="scale">视口当前缩放比例。</param>
        /// <param name="baseCellPixels">网格基准像素大小。</param>
        /// <param name="cellSizeMeters">网格单格物理尺寸（米）。</param>
        public void Render(
            Graphics graphics,
            float offsetX,
            float offsetY,
            float scale,
            float baseCellPixels,
            float cellSizeMeters)
        {
            float pixelsPerMeter = (baseCellPixels * scale) / cellSizeMeters;
            float screenX = offsetX + PositionX * pixelsPerMeter;
            float screenY = offsetY + PositionY * pixelsPerMeter;

            float robotPixelWidth = ChassisWidthMeters * pixelsPerMeter;
            float robotPixelLength = ChassisLengthMeters * pixelsPerMeter;

            GraphicsState state = graphics.Save();
            graphics.SmoothingMode = SmoothingMode.AntiAlias;

            // 平移并旋转坐标系至机器人中心和当前朝向
            graphics.TranslateTransform(screenX, screenY);
            graphics.RotateTransform(CurrentAngle);

            // 绘制底盘投影
            RectangleF chassisRect = new RectangleF(
                -robotPixelLength * 0.5f,
                -robotPixelWidth * 0.5f,
                robotPixelLength,
                robotPixelWidth);

            using (SolidBrush shadowBrush = new SolidBrush(Color.FromArgb(40, 0, 0, 0)))
            {
                RectangleF shadowRect = chassisRect;
                shadowRect.Offset(2f, 2f);
                graphics.FillRectangle(shadowBrush, shadowRect);
            }

            // 绘制机器人主体底盘（物流搬运 AGV 工业黄）
            using (SolidBrush bodyBrush = new SolidBrush(Color.FromArgb(255, 175, 20)))
            using (Pen bodyPen = new Pen(Color.FromArgb(30, 40, 50), 2f))
            {
                graphics.FillRectangle(bodyBrush, chassisRect);
                graphics.DrawRectangle(bodyPen, chassisRect.X, chassisRect.Y, chassisRect.Width, chassisRect.Height);
            }

            // 绘制车头红色方向箭头（指示正前方）
            float arrowLen = robotPixelLength * 0.40f;
            float arrowWid = robotPixelWidth * 0.45f;
            PointF[] arrowPoints = new PointF[]
            {
                new PointF(robotPixelLength * 0.45f, 0f),
                new PointF(robotPixelLength * 0.45f - arrowLen, -arrowWid * 0.5f),
                new PointF(robotPixelLength * 0.45f - arrowLen, arrowWid * 0.5f)
            };

            using (SolidBrush arrowBrush = new SolidBrush(Color.FromArgb(235, 45, 45)))
            {
                graphics.FillPolygon(arrowBrush, arrowPoints);
            }

            // 绘制中心定位旋转点
            float centerRadius = Math.Max(2f, robotPixelWidth * 0.16f);
            using (SolidBrush centerBrush = new SolidBrush(Color.FromArgb(35, 45, 55)))
            {
                graphics.FillEllipse(
                    centerBrush,
                    -centerRadius,
                    -centerRadius,
                    centerRadius * 2f,
                    centerRadius * 2f);
            }

            graphics.Restore(state);
        }
    }
}