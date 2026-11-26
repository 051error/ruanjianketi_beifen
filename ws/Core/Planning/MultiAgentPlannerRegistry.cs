using System;
using System.Collections.Generic;
using System.Linq;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 多智能体路径规划器全局注册中心。
    /// 管理所有已挂载的多机算法实例，支持开箱即用内置算法与运行时动态扩展注册。
    /// </summary>
    public static class MultiAgentPlannerRegistry
    {
        private static readonly Dictionary<string, IMultiAgentPathPlanner> _registry = new(StringComparer.OrdinalIgnoreCase);

        public static IMultiAgentPathPlanner Default => Get("ECBS") ?? GetAll().FirstOrDefault();

        static MultiAgentPlannerRegistry()
        {
            // 静态注册系统内置的 4 大多智能体协同算法
            Register(new EcbsPlanner());
            Register(new CbsPlanner());
            Register(new WhcaStarPlanner());
            Register(new CAStarPlanner());
        }

        public static void Register(IMultiAgentPathPlanner planner)
        {
            // 向全局注册中心挂载并注册多智能体路径规划器。
            // planner: 待注册的规划器实例。
            if (planner == null || string.IsNullOrWhiteSpace(planner.Id)) return;
            _registry[planner.Id] = planner;
        }

        public static IMultiAgentPathPlanner Get(string id)
        {
            // 根据算法唯一标识获取匹配的规划器实例。
            // id: 算法全局唯一英文标识。
            if (string.IsNullOrWhiteSpace(id)) return null;
            _registry.TryGetValue(id, out var planner);
            return planner;
        }

        public static IReadOnlyList<IMultiAgentPathPlanner> GetAll()
        {
            // 获取当前注册中心内所有处于挂载状态的规划器列表。
            return _registry.Values.ToList();
        }
    }
}
