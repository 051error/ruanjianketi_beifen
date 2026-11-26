using System;
using WarehouseGridSimulation.Core.Geometry;

namespace WarehouseGridSimulation.Core.Robots
{
    /// <summary>
    /// 仿真机器人 (AGV) 实体。
    /// </summary>
    public class SimulatedRobot
    {
        public int Id { get; }
        public Pose2D Position { get; private set; }
        public Pose2D PreviousPosition { get; private set; }

        public float LinearVelocity { get; private set; }
        public float AngularVelocity { get; private set; }
        public RobotMotionState MotionState { get; private set; }

        public bool HasCargo { get; set; }
        public int? CurrentTaskId { get; set; }
        public bool HasCollided { get; set; } = false;

        public RobotConfig Config { get; }
        public RobotCommand ActiveCommand { get; private set; } = RobotCommand.Stop;

        public float RemainingDistance { get; private set; }
        public float RemainingAngle { get; private set; }

        public GridPoint ConfirmedGrid { get; private set; }
        public GridPoint GridPosition => ConfirmedGrid;

        private float _cellSizeMeters;

        public SimulatedRobot(int id, float x, float y, float heading, float cellSizeMeters, RobotConfig config = null)
        {
            // 初始化机器人仿真动力学实体。
            // id: 机器人全局唯一标识编号; x: 初始世界横坐标; y: 初始世界纵坐标; heading: 初始航向角; cellSizeMeters: 地图单网格物理尺寸; config: 动力学参数配置。
            Id = id;
            Config = config ?? new RobotConfig();
            _cellSizeMeters = cellSizeMeters > 0f ? cellSizeMeters : 0.55f;
            Position = new Pose2D(x, y, heading);
            PreviousPosition = Position;
            ConfirmedGrid = new GridPoint((int)Math.Floor(x / _cellSizeMeters), (int)Math.Floor(y / _cellSizeMeters));
            LinearVelocity = 0f;
            AngularVelocity = 0f;
            RemainingDistance = 0f;
            RemainingAngle = 0f;
            MotionState = RobotMotionState.Stopped;
            HasCargo = false;
            CurrentTaskId = null;
        }

        public SimulatedRobot(int id, GridPoint gridPoint, float heading, float cellSizeMeters, RobotConfig config = null)
            : this(id, (gridPoint.X + 0.5f) * cellSizeMeters, (gridPoint.Y + 0.5f) * cellSizeMeters, heading, cellSizeMeters, config)
        {
            // 基于离散栅格坐标重载初始化机器人仿真实体。
            // gridPoint: 初始所处离散栅格坐标。
        }

        public void ExecuteCommand(RobotCommand command)
        {
            // 接收并设置外部运动控制器下发的基础动力学信号。
            // command: 基础控制指令。
            ActiveCommand = command;
            switch (command.Type)
            {
                case RobotCommandType.MoveDistance:
                    // 转向与前进严格互斥：自旋角速度未静止前拒绝接收前进位移
                    if (Math.Abs(AngularVelocity) <= 0.02f)
                    {
                        RemainingDistance = command.Distance;
                        RemainingAngle = 0f;
                    }
                    break;
                case RobotCommandType.RotateAngle:
                    // 转向与前进严格互斥：线速度未静止前拒绝接收转向角度
                    if (LinearVelocity <= Config.SpeedTolerance)
                    {
                        RemainingAngle = command.Angle;
                        RemainingDistance = 0f;
                    }
                    break;
                case RobotCommandType.Stop:
                default:
                    RemainingDistance = 0f;
                    RemainingAngle = 0f;
                    break;
            }
        }

        public void EmergencyStop()
        {
            // 执行紧急制动并切断驱动动力。
            ActiveCommand = RobotCommand.Stop;
            RemainingDistance = 0f;
            RemainingAngle = 0f;
            LinearVelocity = 0f;
            AngularVelocity = 0f;
            MotionState = RobotMotionState.Stopped;
        }

        public void Step(float dt, float cellSizeMeters)
        {
            // 推进执行单仿真步长的连续动力学状态计算与中心地标识别。
            // dt: 仿真时间微元步长。
            if (dt <= 0f) return;
            _cellSizeMeters = cellSizeMeters;
            PreviousPosition = Position;

            float currentX = Position.X;
            float currentY = Position.Y;
            float currentHeading = Position.Heading;

            if (ActiveCommand.Type == RobotCommandType.MoveDistance && RemainingDistance > 0f)
            {
                // 1. 前进 (位移) 信号执行：转向与前进严格互斥，角速度强制为 0
                AngularVelocity = 0f;

                // 梯形减速前瞻：计算以最大减速度刹停所需的制动滑行距离
                float stoppingDist = (LinearVelocity * LinearVelocity) / (2f * Config.LinearDeceleration);
                if (RemainingDistance <= stoppingDist && LinearVelocity > 0.3f)
                {
                    LinearVelocity = Math.Max(0.2f, LinearVelocity - Config.LinearDeceleration * dt);
                }
                else
                {
                    LinearVelocity = Math.Min(Config.MaxLinearSpeed, LinearVelocity + Config.LinearAcceleration * dt);
                }

                float stepDisplacement = LinearVelocity * dt;
                if (stepDisplacement >= RemainingDistance)
                {
                    stepDisplacement = RemainingDistance;
                    RemainingDistance = 0f;
                    LinearVelocity = 0f;
                    ActiveCommand = RobotCommand.Stop;
                }
                else
                {
                    RemainingDistance -= stepDisplacement;
                }

                currentX += stepDisplacement * (float)Math.Cos(currentHeading);
                currentY += stepDisplacement * (float)Math.Sin(currentHeading);
                MotionState = (LinearVelocity > 0f) ? RobotMotionState.MovingForward : RobotMotionState.Stopped;
            }
            else if (ActiveCommand.Type == RobotCommandType.RotateAngle && Math.Abs(RemainingAngle) > 0f)
            {
                // 2. 转向 (角度) 信号执行：转向与前进严格互斥，线速度强制为 0
                LinearVelocity = 0f;
                float turnDir = RemainingAngle > 0f ? 1f : -1f;
                float curAbsOmega = Math.Abs(AngularVelocity);

                // 梯形角减速前瞻：计算以最大角减速度刹停所需的制动滑行角位移
                float stoppingAngle = (curAbsOmega * curAbsOmega) / (2f * Config.AngularDeceleration);
                if (Math.Abs(RemainingAngle) <= stoppingAngle && curAbsOmega > 0.3f)
                {
                    curAbsOmega = Math.Max(0.2f, curAbsOmega - Config.AngularDeceleration * dt);
                }
                else
                {
                    curAbsOmega = Math.Min(Config.TurnSpeed, curAbsOmega + Config.AngularAcceleration * dt);
                }

                AngularVelocity = turnDir * curAbsOmega;
                float stepAngle = curAbsOmega * dt;

                if (stepAngle >= Math.Abs(RemainingAngle))
                {
                    currentHeading = Pose2D.NormalizeAngle(currentHeading + RemainingAngle);
                    RemainingAngle = 0f;
                    AngularVelocity = 0f;
                    ActiveCommand = RobotCommand.Stop;
                }
                else
                {
                    currentHeading = Pose2D.NormalizeAngle(currentHeading + turnDir * stepAngle);
                    RemainingAngle -= turnDir * stepAngle;
                }

                MotionState = (turnDir > 0f) ? RobotMotionState.TurningRight : RobotMotionState.TurningLeft;
            }
            else
            {
                // 3. 停止 (制动) 信号执行：严格按减速度物理滑行制动停下 (方案 A)
                LinearVelocity = Math.Max(0f, LinearVelocity - Config.LinearDeceleration * dt);
                if (AngularVelocity > 0f)
                {
                    AngularVelocity = Math.Max(0f, AngularVelocity - Config.AngularDeceleration * dt);
                }
                else if (AngularVelocity < 0f)
                {
                    AngularVelocity = Math.Min(0f, AngularVelocity + Config.AngularDeceleration * dt);
                }

                if (LinearVelocity > 0f)
                {
                    float displacement = LinearVelocity * dt;
                    currentX += displacement * (float)Math.Cos(currentHeading);
                    currentY += displacement * (float)Math.Sin(currentHeading);
                    MotionState = RobotMotionState.MovingForward;
                }
                else if (Math.Abs(AngularVelocity) > 0.05f)
                {
                    currentHeading = Pose2D.NormalizeAngle(currentHeading + AngularVelocity * dt);
                    MotionState = (AngularVelocity > 0f) ? RobotMotionState.TurningRight : RobotMotionState.TurningLeft;
                }
                else
                {
                    LinearVelocity = 0f;
                    AngularVelocity = 0f;
                    MotionState = RobotMotionState.Stopped;
                }
            }

            Position = new Pose2D(currentX, currentY, currentHeading);

            // 核心约束：只有移动到栅格正中央 (距离中心 <= PositionTolerance) 才能知道当前所在栅格 (模拟地面地标读码)
            int gx = (int)Math.Floor(Position.X / cellSizeMeters);
            int gy = (int)Math.Floor(Position.Y / cellSizeMeters);
            float targetCenterX = (gx + 0.5f) * cellSizeMeters;
            float targetCenterY = (gy + 0.5f) * cellSizeMeters;
            float distToCenter = Position.DistanceTo(targetCenterX, targetCenterY);

            if (distToCenter <= Config.PositionTolerance)
            {
                ConfirmedGrid = new GridPoint(gx, gy);
            }
        }

        public void TeleportForReset(Pose2D pose)
        {
            // 在重置或初始部署时安全复位机器人位姿与确认网格。
            // pose: 目标绝对重置位姿。
            Position = pose;
            PreviousPosition = pose;
            ConfirmedGrid = new GridPoint((int)Math.Floor(pose.X / _cellSizeMeters), (int)Math.Floor(pose.Y / _cellSizeMeters));
            LinearVelocity = 0f;
            AngularVelocity = 0f;
            HasCollided = false;
            ActiveCommand = RobotCommand.Stop;
            MotionState = RobotMotionState.Stopped;
        }

        public override string ToString()
        {
            // 输出格式化的机器人状态字符串。
            return $"Robot #{Id} at {ConfirmedGrid} ({Position.X:F2},{Position.Y:F2}) Vel={LinearVelocity:F2} State={MotionState} Cargo={HasCargo}";
        }
    }
}
