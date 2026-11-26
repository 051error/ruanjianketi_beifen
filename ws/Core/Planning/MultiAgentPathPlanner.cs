using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Map;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 默认多智能体路径规划代理器。
    /// 统一基于 MultiAgentPlannerRegistry 注册中心代理分发调度。
    /// </summary>
    public class MultiAgentPathPlanner : IMultiAgentPathPlanner
    {
        public string Id => "Default";
        public string Name => "默认多机调度规划器";

        public IMultiAgentPathPlanner ActivePlanner { get; set; }

        public MultiAgentPathPlanner(IMultiAgentPathPlanner activePlanner = null)
        {
            // 初始化默认多机路径规划代理器。
            ActivePlanner = activePlanner ?? MultiAgentPlannerRegistry.Default;
        }

        public MultiAgentPlanResult Plan(
            GridMap map,
            IReadOnlyList<AgentTask> tasks,
            MultiAgentPlanOptions options = null)
        {
            // 委托当前活动规划器执行多机路径规划。
            // map: 仓储栅格地图; tasks: 参与调度的智能体任务清单; options: 算法配置选项。
            var planner = ActivePlanner ?? MultiAgentPlannerRegistry.Default;
            return planner.Plan(map, tasks, options);
        }
    }
}
