namespace WarehouseGridSimulation.Core.Engine
{
    /// <summary>
    /// 仿真引擎配置参数。
    /// </summary>
    public class SimulationConfig
    {
        /// <summary>
        /// 统一、固定的仿真时间步长 (秒)，默认 0.1 秒 (10 Hz)。
        /// </summary>
        public float FixedDeltaTime { get; set; } = 0.1f;

        /// <summary>
        /// 发生独立碰撞时是否自动暂停仿真。
        /// </summary>
        public bool AutoPauseOnCollision { get; set; } = true;

        /// <summary>
        /// 机器人单段路径授权预看步数 (默认每次向前申请 1~2 步)。
        /// </summary>
        public int LookaheadAuthorizationSteps { get; set; } = 1;
    }
}
