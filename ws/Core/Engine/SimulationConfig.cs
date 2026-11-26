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
        /// 机器人单段路径授权前瞻步数 (基于动力学最大制动距离 2 格 + 2 格安全冗余，统一为 4 格移动闭塞区间)。
        /// </summary>
        public int LookaheadAuthorizationSteps { get; set; } = 4;

        /// <summary>
        /// 检测到循环等待死锁时是否自动触发在线协同重规划自适应脱困。
        /// </summary>
        public bool AutoReplanningOnDeadlock { get; set; } = true;
    }
}
