using System;
using SkiaSharp;

namespace WarehouseGridSimulation.Skia.Models
{
    /// <summary>
    /// 表示机器人在离散栅格中的四正交朝向。
    /// </summary>
    public enum RobotOrientation
    {
        Up = 0,
        Right = 1,
        Down = 2,
        Left = 3
    }

    /// <summary>
    /// 表示机器人当前的动作状态。
    /// </summary>
    public enum RobotState
    {
        Idle = 0,
        Moving = 1,
        Turning = 2
    }

    /// <summary>
    /// 表示离散仓储栅格中运动的单个机器人（AGV），采用 SkiaSharp 进行高质量向量渲染。
    /// </summary>
    public class Robot
    {
        public const float ChassisLengthMeters = 0.42f;
        public const float ChassisWidthMeters = 0.38f;

        public int GridX { get; set; }
        public int GridY { get; set; }
        public int TargetGridX { get; set; }
        public int TargetGridY { get; set; }
        public RobotOrientation Orientation { get; set; }
        public RobotState State { get; private set; }

        public float PositionX { get; private set; }
        public float PositionY { get; private set; }
        public float CurrentAngle { get; private set; }

        public float MaxSpeed { get; set; }
        public float Acceleration { get; set; }
        public float CurrentSpeed { get; private set; }

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

        public Robot(int gridX, int gridY, float cellSizeMeters, RobotOrientation orientation)
        {
            // 初始化机器人实例并对齐至指定离散网格中心
            // gridX: 初始网格列索引
            // gridY: 初始网格行索引
            // cellSizeMeters: 单个网格物理边长（米）
            // orientation: 初始正交朝向
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

        public static float GetBaseAngle(RobotOrientation orientation)
        {
            // 将正交朝向枚举转换为标准的度数值
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

        public bool TryMoveForward(int gridColumns, int gridRows, float cellSizeMeters)
        {
            // 请求向当前正交朝向前进一格并规划梯形速度曲线
            // gridColumns: 地图网格总列数
            // gridRows: 地图网格总行数
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

            float safeAccel = Math.Max(0.1f, Acceleration);
            float safeMaxSpeed = Math.Max(0.1f, MaxSpeed);

            float accelDist = (safeMaxSpeed * safeMaxSpeed) / (2.0f * safeAccel);
            if (accelDist * 2.0f >= _moveDistanceMeters)
            {
                _peakSpeed = (float)Math.Sqrt(safeAccel * _moveDistanceMeters);
                _accelTime = _peakSpeed / safeAccel;
                _constTime = 0f;
                _totalMoveTime = _accelTime * 2.0f;
            }
            else
            {
                _peakSpeed = safeMaxSpeed;
                _accelTime = _peakSpeed / safeAccel;
                float constDist = _moveDistanceMeters - accelDist * 2.0f;
                _constTime = constDist / _peakSpeed;
                _totalMoveTime = _accelTime * 2.0f + _constTime;
            }

            State = RobotState.Moving;
            return true;
        }

        public bool TryTurnLeft()
        {
            // 请求机器人逆时针原地旋转90度
            if (State != RobotState.Idle)
            {
                return false;
            }

            RobotOrientation newOrientation = (RobotOrientation)(((int)Orientation - 1 + 4) % 4);
            StartTurning(newOrientation, -90f);
            return true;
        }

        public bool TryTurnRight()
        {
            // 请求机器人顺时针原地旋转90度
            if (State != RobotState.Idle)
            {
                return false;
            }

            RobotOrientation newOrientation = (RobotOrientation)(((int)Orientation + 1) % 4);
            StartTurning(newOrientation, 90f);
            return true;
        }

        private void StartTurning(RobotOrientation newOrientation, float deltaAngle)
        {
            // 设定目标朝向并开启90度角平滑过渡动画
            // newOrientation: 旋转完成后的目标朝向
            // deltaAngle: 角度偏移量，负值为左转，正值为右转
            Orientation = newOrientation;
            _startAngle = CurrentAngle;
            _targetAngle = CurrentAngle + deltaAngle;
            _turnElapsed = 0f;
            _turnDuration = 0.25f;
            State = RobotState.Turning;
        }

        public void ResetToGrid(int gridX, int gridY, float cellSizeMeters)
        {
            // 将机器人瞬移重置回指定网格中心并恢复空闲状态
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

        public void Update(float deltaTime, float cellSizeMeters)
        {
            // 根据时间流逝更新机器人的位置插值与转向插值
            // deltaTime: 自上一帧以来的时间步长（秒）
            if (deltaTime <= 0f)
            {
                return;
            }

            if (State == RobotState.Turning)
            {
                _turnElapsed += deltaTime;
                if (_turnElapsed >= _turnDuration)
                {
                    CurrentAngle = GetBaseAngle(Orientation);
                    _targetAngle = CurrentAngle;
                    State = RobotState.Idle;
                }
                else
                {
                    float progress = _turnElapsed / _turnDuration;
                    float smooth = progress * progress * (3.0f - 2.0f * progress);
                    CurrentAngle = _startAngle + (_targetAngle - _startAngle) * smooth;
                }
            }
            else if (State == RobotState.Moving)
            {
                _moveElapsed += deltaTime;
                if (_moveElapsed >= _totalMoveTime)
                {
                    GridX = TargetGridX;
                    GridY = TargetGridY;
                    PositionX = (GridX + 0.5f) * cellSizeMeters;
                    PositionY = (GridY + 0.5f) * cellSizeMeters;
                    CurrentSpeed = 0f;
                    State = RobotState.Idle;
                }
                else
                {
                    float s = 0f;
                    float safeAccel = Math.Max(0.1f, Acceleration);

                    if (_moveElapsed < _accelTime)
                    {
                        s = 0.5f * safeAccel * _moveElapsed * _moveElapsed;
                        CurrentSpeed = safeAccel * _moveElapsed;
                    }
                    else if (_moveElapsed < _accelTime + _constTime)
                    {
                        float accelDist = 0.5f * safeAccel * _accelTime * _accelTime;
                        float constElapsed = _moveElapsed - _accelTime;
                        s = accelDist + _peakSpeed * constElapsed;
                        CurrentSpeed = _peakSpeed;
                    }
                    else
                    {
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
                PositionX = (GridX + 0.5f) * cellSizeMeters;
                PositionY = (GridY + 0.5f) * cellSizeMeters;
            }
        }

        public void Render(
            SKCanvas canvas,
            float offsetX,
            float offsetY,
            float scale,
            float baseCellPixels,
            float cellSizeMeters)
        {
            // 使用 SkiaSharp 高质量向量渲染机器人底盘、车头箭头与中心枢轴
            // canvas: Skia 绘图画布
            // offsetX: 视口 X 轴像素偏移
            // offsetY: 视口 Y 轴像素偏移
            // scale: 视口缩放系数
            // baseCellPixels: 基础网格像素大小
            float pixelsPerMeter = (baseCellPixels * scale) / cellSizeMeters;
            float screenX = offsetX + PositionX * pixelsPerMeter;
            float screenY = offsetY + PositionY * pixelsPerMeter;

            float robotPixelWidth = ChassisWidthMeters * pixelsPerMeter;
            float robotPixelLength = ChassisLengthMeters * pixelsPerMeter;

            canvas.Save();
            canvas.Translate(screenX, screenY);
            canvas.RotateDegrees(CurrentAngle);

            SKRect chassisRect = new SKRect(
                -robotPixelLength * 0.5f,
                -robotPixelWidth * 0.5f,
                robotPixelLength * 0.5f,
                robotPixelWidth * 0.5f);

            float cornerRadius = Math.Max(2f, robotPixelWidth * 0.12f);

            // 1. 绘制机器人地面投影阴影
            using (var shadowPaint = new SKPaint
            {
                Color = new SKColor(0, 0, 0, 50),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                SKRect shadowRect = chassisRect;
                shadowRect.Offset(2f * scale, 2f * scale);
                canvas.DrawRoundRect(shadowRect, cornerRadius, cornerRadius, shadowPaint);
            }

            // 2. 绘制 AGV 主体底盘（亮工业黄，带圆角）
            using (var bodyPaint = new SKPaint
            {
                Color = new SKColor(255, 175, 20),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                canvas.DrawRoundRect(chassisRect, cornerRadius, cornerRadius, bodyPaint);
            }

            // 3. 绘制底盘黑色精细边框
            using (var strokePaint = new SKPaint
            {
                Color = new SKColor(35, 45, 55),
                IsAntialias = true,
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(1.5f, 2f * scale)
            })
            {
                canvas.DrawRoundRect(chassisRect, cornerRadius, cornerRadius, strokePaint);
            }

            // 4. 绘制车头红色方向箭头（指示 X 轴正前方）
            float arrowLen = robotPixelLength * 0.40f;
            float arrowWid = robotPixelWidth * 0.45f;
            using (var arrowPath = new SKPath())
            using (var arrowPaint = new SKPaint
            {
                Color = new SKColor(235, 45, 45),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                arrowPath.MoveTo(robotPixelLength * 0.45f, 0f);
                arrowPath.LineTo(robotPixelLength * 0.45f - arrowLen, -arrowWid * 0.5f);
                arrowPath.LineTo(robotPixelLength * 0.45f - arrowLen, arrowWid * 0.5f);
                arrowPath.Close();
                canvas.DrawPath(arrowPath, arrowPaint);
            }

            // 5. 绘制中心定位旋转点
            float centerRadius = Math.Max(2f, robotPixelWidth * 0.16f);
            using (var centerPaint = new SKPaint
            {
                Color = new SKColor(40, 50, 60),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                canvas.DrawCircle(0f, 0f, centerRadius, centerPaint);
            }

            canvas.Restore();
        }
    }
}
