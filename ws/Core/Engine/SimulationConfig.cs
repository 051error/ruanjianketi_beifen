using WarehouseGridSimulation.Core.Traffic;

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

        /// <summary>
        /// 三格移动闭塞延迟释放策略模式 (默认到达目标点后释放除终点外的所有资源，作为当前多机调度的预约机制)。
        /// </summary>
        public ThreeGridDelayReleaseMode DelayReleaseMode { get; set; } = ThreeGridDelayReleaseMode.ReleaseAllExceptDestinationOnArrival;

        /// <summary>
        /// 局部死锁自愈无解时，是否允许自动升级触发全局全机群时空协同重规划 (Tier 3 保底机制)。
        /// </summary>
        public bool EnableGlobalReplanningOnDeadlock { get; set; } = true;

        /// <summary>
        /// 全局协同重规划触发防抖冷却时间 (秒)，避免高密度拥堵下连续高频触发全局重算。
        /// </summary>
        public float GlobalReplanningCooldownSeconds { get; set; } = 2.0f;

        /// <summary>
        /// 全局多智能体协同重规划求解超时上限 (毫秒)，提供充裕的时空协调求解预算。
        /// </summary>
        public int GlobalReplanningTimeoutMs { get; set; } = 3500;

        /// <summary>
        /// 是否启用后台独立工作线程自驱动物理仿真步进 (UI 宿主模式下启用，与渲染线程彻底解耦；离散单步测试模式下可禁用)。
        /// </summary>
        public bool EnableBackgroundWorker { get; set; } = false;
    }
}
