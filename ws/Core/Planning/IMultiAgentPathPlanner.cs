using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 单个机器人的多机调度任务契约。
    /// </summary>
    public struct AgentTask
    {
        public int AgentId;
        public GridPoint Start;
        public GridPoint Target;

        public AgentTask(int agentId, GridPoint start, GridPoint target)
        {
            AgentId = agentId;
            Start = start;
            Target = target;
        }

        public override string ToString() => $"Agent #{AgentId}: {Start} -> {Target}";
    }

    /// <summary>
    /// 包含连续时间区间的时空路径节点。
    /// </summary>
    public struct TimedPathPoint
    {
        public GridPoint Grid;
        public double PlannedEntryTime; // 计划进入该网格的开始时刻 (秒)
        public double PlannedExitTime;  // 计划完全离开该网格的时刻 (秒，含保守安全缓冲)

        public TimedPathPoint(GridPoint grid, double entryTime, double exitTime)
        {
            Grid = grid;
            PlannedEntryTime = entryTime;
            PlannedExitTime = exitTime;
        }

        public override string ToString() => $"{Grid} @ [{PlannedEntryTime:F2}s - {PlannedExitTime:F2}s]";
    }

    /// <summary>
    /// 保守占用时间区间。
    /// </summary>
    public struct TimeInterval
    {
        public double EntryTime;
        public double ExitTime;
        public int AgentId;

        public TimeInterval(double entry, double exit, int agentId)
        {
            EntryTime = entry;
            ExitTime = exit;
            AgentId = agentId;
        }

        public bool Overlaps(double otherEntry, double otherExit)
        {
            return Math.Max(EntryTime, otherEntry) < Math.Min(ExitTime, otherExit);
        }
    }

    /// <summary>
    /// 多智能体路径规划参数配置。
    /// </summary>
    public class MultiAgentPlanOptions
    {
        public int TimeoutMilliseconds { get; set; } = 4000;
        public int WhcaWindowK { get; set; } = 18;
        public double EcbsSuboptimality { get; set; } = 1.30;
        public Func<GridPoint, bool> IsDynamicBlocked { get; set; }
    }

    /// <summary>
    /// 多智能体调度规划输出结果。
    /// </summary>
    public sealed class MultiAgentPlanResult
    {
        public bool Success { get; set; }
        public bool IsTimeout { get; set; }
        public List<List<GridPoint>> AgentPaths { get; set; }
        public List<List<TimedPathPoint>> TimedSchedules { get; set; }
        public string Message { get; set; }
        public long ElapsedMilliseconds { get; set; }

        public MultiAgentPlanResult()
        {
            AgentPaths = new List<List<GridPoint>>();
            TimedSchedules = new List<List<TimedPathPoint>>();
            Message = string.Empty;
        }

        public static MultiAgentPlanResult Fail(string message, bool isTimeout = false)
        {
            return new MultiAgentPlanResult
            {
                Success = false,
                IsTimeout = isTimeout,
                Message = message
            };
        }
    }

    /// <summary>
    /// 多智能体路径规划器接口。
    /// </summary>
    public interface IMultiAgentPathPlanner
    {
        /// <summary>
        /// 规划算法全局唯一英文标识 (例如 "CBS", "ECBS", "CAStar", "WHCAStar")。
        /// </summary>
        string Id { get; }

        /// <summary>
        /// 规划算法界面展示友好名称 (例如 "ECBS (增强型CBS快速算法)")。
        /// </summary>
        string Name { get; }

        MultiAgentPlanResult Plan(
            GridMap map,
            IReadOnlyList<AgentTask> tasks,
            MultiAgentPlanOptions options = null);
    }
}
