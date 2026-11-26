using System;

namespace WarehouseGridSimulation.Core.Robots
{
    /// <summary>
    /// 机器人物理特性与运动学动力学参数配置。
    /// 二维运动模型说明：
    /// 1. 外形尺寸：长 0.42m，宽 0.38m，外包络碰撞等效圆半径为 0.25m。
    /// 2. 原地转向：采用差速轮模型，支持原地旋转 (TurnLeft/TurnRight)，旋转时线速度制动归零。
    /// 3. 加减速：前进时具有固定线加速度与制动减速度，遵循动力学加减速曲线，非瞬态突变。
    /// 4. 停靠容差：位置容差 PositionTolerance (0.05m)，航向容差 HeadingTolerance (0.05rad ≈ 2.8°)，速度容差 SpeedTolerance (0.05m/s)。
    /// </summary>
    public class RobotConfig
    {
        public float MaxLinearSpeed { get; set; } = 1.5f; // 最大线速度 (m/s)
        public float LinearAcceleration { get; set; } = 1.5f; // 最大线加速度 (m/s^2)
        public float LinearDeceleration { get; set; } = 2.0f; // 线减速度 (m/s^2)
        public float TurnSpeed { get; set; } = (float)Math.PI; // 最大角速度 (rad/s, 180°/s)
        public float AngularAcceleration { get; set; } = 2f * (float)Math.PI; // 最大角加速度 (rad/s^2, 360°/s^2)
        public float AngularDeceleration { get; set; } = 2f * (float)Math.PI; // 最大角减速度 (rad/s^2, 360°/s^2)

        public float ChassisLength { get; set; } = 0.42f; // 底盘长度 (米)
        public float ChassisWidth { get; set; } = 0.38f; // 底盘宽度 (米)
        public float CollisionRadius { get; set; } = 0.25f; // 碰撞检测包络圆半径 (米)

        public float PositionTolerance { get; set; } = 0.1f; // 到位距离容差 (米)
        public float HeadingTolerance { get; set; } = 0.01f; // 航向角对齐容差 (弧度，约0.57°高精度判停)
        public float SpeedTolerance { get; set; } = 0.05f; // 停止判定速度容差 (m/s)

        public float CreepThreshold { get; set; } = 0.08f; // 切换到闭环精对准的剩余距离门限 (米)
        public float CreepMaxSpeed { get; set; } = 0.2f; // 闭环低速对准阶段最大爬行线速度 (m/s)
        public float AlignmentKp { get; set; } = 2.0f; // 到站对齐纵向比例增益
        public float AlignmentKd { get; set; } = 0.1f; // 到站对齐纵向微分增益

        public RobotConfig Clone()
        {
            // 创建并返回当前配置对象的浅表副本。
            return (RobotConfig)MemberwiseClone();
        }
    }
}
