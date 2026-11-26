using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Map;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 窗口协同 A* (Windowed Hierarchical CA*, WHCA*) 多智能体路径规划器。
    /// 基于滑动时空预订窗口，支持大规模、长程流水线多智能体快速协同调度。
    /// </summary>
    public class WhcaStarPlanner : CAStarPlanner
    {
        public override string Id => "WHCAStar";
        public override string Name => "WHCA* (窗口协同A*算法)";

        public override MultiAgentPlanResult Plan(
            GridMap map,
            IReadOnlyList<AgentTask> tasks,
            MultiAgentPlanOptions options = null)
        {
            // 执行窗口化分层协同 A* 多智能体时空路径规划。
            // map: 仓储栅格地图; tasks: 参与调度的智能体任务清单; options: 算法配置选项。
            var opt = options ?? new MultiAgentPlanOptions();
            int windowSize = Math.Max(4, opt.WhcaWindowK);
            return ExecutePlan(map, tasks, opt, windowSize);
        }
    }
}
