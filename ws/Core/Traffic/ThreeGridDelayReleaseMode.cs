namespace WarehouseGridSimulation.Core.Traffic
{
    /// <summary>
    /// 三格移动闭塞延迟释放策略模式。
    /// </summary>
    public enum ThreeGridDelayReleaseMode
    {
        /// <summary>
        /// 到达目标点后不释放上一栅格资源，持续保持车身后方延迟闭塞锁定 (供后续 LMAPF 场景使用)。
        /// </summary>
        RetainPreviousGridOnDestination = 0,

        /// <summary>
        /// 到达目标点后释放除当前目标点外的所有资源 (作为当前多机调度的预约机制)。
        /// </summary>
        ReleaseAllExceptDestinationOnArrival = 1
    }
}
