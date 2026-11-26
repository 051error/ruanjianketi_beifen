using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 增强型基于冲突搜索算法 (Enhanced CBS, ECBS) 多智能体路径规划器。
    /// 在 CBS 约束树基础上引入焦点次优启发式搜索，实现大规模多机冲突的毫秒级快速消解。
    /// </summary>
    public class EcbsPlanner : CbsPlanner
    {
        public override string Id => "ECBS";
        public override string Name => "ECBS (增强型CBS快速算法)";

        public override MultiAgentPlanResult Plan(
            GridMap map,
            IReadOnlyList<AgentTask> tasks,
            MultiAgentPlanOptions options = null)
        {
            // 执行增强型冲突搜索 (ECBS) 次优多智能体路径规划。
            // map: 仓储栅格地图; tasks: 参与调度的智能体任务清单; options: 算法配置选项。
            var opt = options ?? new MultiAgentPlanOptions();
            double suboptimality = Math.Max(1.0, opt.EcbsSuboptimality);
            return ExecutePlan(map, tasks, opt, isEnhanced: true, suboptimality: suboptimality);
        }
    }
}
