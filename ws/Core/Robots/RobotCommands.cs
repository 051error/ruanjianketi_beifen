using System;

namespace WarehouseGridSimulation.Core.Robots
{
    /// <summary>
    /// 机器人支持的基础运动信号类型：前进（位移）、转向（角度）、停止（制动）。
    /// </summary>
    public enum RobotCommandType
    {
        /// <summary>
        /// 沿当前航向前进指定位移距离 (单位: 米)。
        /// </summary>
        MoveDistance = 0,

        /// <summary>
        /// 原地自旋指定旋转角度 (单位: 弧度，正值右转/顺时针，负值左转/逆时针)。
        /// </summary>
        RotateAngle = 1,

        /// <summary>
        /// 减速制动并保持静止。
        /// </summary>
        Stop = 2
    }

    /// <summary>
    /// 机器人基础控制信号结构。
    /// </summary>
    public readonly struct RobotCommand
    {
        public RobotCommandType Type { get; }
        public float Parameter { get; }

        public float Distance => (Type == RobotCommandType.MoveDistance) ? Parameter : 0f;
        public float Angle => (Type == RobotCommandType.RotateAngle) ? Parameter : 0f;

        public RobotCommand(RobotCommandType type, float parameter = 0f)
        {
            // 初始化机器人定量基础控制信号。
            // type: 信号类型; parameter: 携带的位移量或角度量。
            Type = type;
            Parameter = parameter;
        }

        public static RobotCommand Move(float distance)
        {
            // 构建指定位移距离的前进信号。
            // distance: 期望行进的位移距离 (米)。
            return new RobotCommand(RobotCommandType.MoveDistance, Math.Max(0f, distance));
        }

        public static RobotCommand Rotate(float angle)
        {
            // 构建指定旋转角度的原地自旋信号。
            // angle: 期望自旋的角度弧度值。
            return new RobotCommand(RobotCommandType.RotateAngle, angle);
        }

        public static RobotCommand Brake()
        {
            // 构建立即减速停靠的制动信号。
            return new RobotCommand(RobotCommandType.Stop, 0f);
        }

        public static RobotCommand Stop => Brake();
        public static RobotCommand MoveForward => Move(float.MaxValue);
        public static RobotCommand TurnLeft => Rotate(-(float)Math.PI * 0.5f);
        public static RobotCommand TurnRight => Rotate((float)Math.PI * 0.5f);

        public override string ToString()
        {
            // 格式化输出信号类型与参数描述。
            return Type switch
            {
                RobotCommandType.MoveDistance => $"Move({Distance:F2}m)",
                RobotCommandType.RotateAngle => $"Rotate({Angle * 180f / Math.PI:F1}°)",
                _ => "Stop(Brake)"
            };
        }
    }

    /// <summary>
    /// 机器人当前运动状态。
    /// </summary>
    public enum RobotMotionState
    {
        Stopped = 0,
        MovingForward = 1,
        TurningLeft = 2,
        TurningRight = 3
    }
}
