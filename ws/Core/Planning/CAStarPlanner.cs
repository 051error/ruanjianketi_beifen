using System;
using System.Collections.Generic;
using System.Diagnostics;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 协同 A* (Cooperative A*) 多智能体路径规划器。
    /// 基于全局连续保守时间区间预约表按优先级顺序执行无冲突时空规划。
    /// </summary>
    public class CAStarPlanner : IMultiAgentPathPlanner
    {
        public virtual string Id => "CAStar";
        public virtual string Name => "CA* (协同A*算法)";

        public virtual MultiAgentPlanResult Plan(
            GridMap map,
            IReadOnlyList<AgentTask> tasks,
            MultiAgentPlanOptions options = null)
        {
            // 执行协同 A* 多智能体时空路径规划。
            // map: 仓储栅格地图; tasks: 参与调度的智能体任务清单; options: 算法配置选项。
            var opt = options ?? new MultiAgentPlanOptions();
            return ExecutePlan(map, tasks, opt, windowSize: 0);
        }

        protected MultiAgentPlanResult ExecutePlan(
            GridMap map,
            IReadOnlyList<AgentTask> tasks,
            MultiAgentPlanOptions opt,
            int windowSize)
        {
            // 执行带保守时间区间预约表的时空规划底层流转。
            // opt: 规划参数选项; windowSize: 时空滑动窗口大小。
            var sw = Stopwatch.StartNew();
            var res = new MultiAgentPlanResult();

            if (tasks == null || tasks.Count == 0)
            {
                res.Success = true;
                res.Message = "无任务需求";
                return res;
            }

            int n = tasks.Count;
            int gridCols = map?.Width ?? 30;
            int gridRows = map?.Height ?? 30;

            var gridIntervals = new Dictionary<int, List<TimeInterval>>();
            var edgeIntervals = new Dictionary<long, List<EdgeInterval>>();

            for (int a = 0; a < n; a++)
            {
                int sKey = GetCellKey(tasks[a].Start.X, tasks[a].Start.Y);
                if (!gridIntervals.TryGetValue(sKey, out var list))
                {
                    list = new List<TimeInterval>();
                    gridIntervals[sKey] = list;
                }
                list.Add(new TimeInterval(0.0, 1000.0, a));
            }

            bool IsConflict(int agentId, GridPoint from, GridPoint to, double enterTime, double leaveTime)
            {
                for (int j = 0; j < n; j++)
                {
                    if (j != agentId && tasks[j].Target == to && tasks[agentId].Target != to)
                    {
                        return true;
                    }
                }

                if (windowSize > 0 && enterTime > windowSize * SpaceTimeAStar.StepPhysicalDuration)
                {
                    return false;
                }

                int toKey = GetCellKey(to.X, to.Y);
                if (gridIntervals.TryGetValue(toKey, out var list))
                {
                    for (int k = 0; k < list.Count; k++)
                    {
                        var iv = list[k];
                        if (iv.AgentId != agentId && iv.Overlaps(enterTime, leaveTime))
                        {
                            return true;
                        }
                    }
                }

                if (from != to)
                {
                    long edgeKey = GetUndirectedEdgeKey(from, to);
                    if (edgeIntervals.TryGetValue(edgeKey, out var eList))
                    {
                        for (int k = 0; k < eList.Count; k++)
                        {
                            var eIv = eList[k];
                            if (eIv.AgentId != agentId && eIv.Overlaps(enterTime, leaveTime))
                            {
                                return true;
                            }
                        }
                    }
                }

                return false;
            }

            for (int i = 0; i < n; i++)
            {
                if (sw.ElapsedMilliseconds > opt.TimeoutMilliseconds)
                {
                    res.Success = false;
                    res.IsTimeout = true;
                    res.Message = "CA* 规划超时！";
                    res.ElapsedMilliseconds = sw.ElapsedMilliseconds;
                    return res;
                }

                var (p, tp) = SpaceTimeAStar.SearchPath(
                    map,
                    tasks[i].Start,
                    tasks[i].Target,
                    i,
                    IsConflict,
                    SpaceTimeAStar.MaxTimeSteps,
                    suboptimality: 1.0,
                    isDynamicBlocked: opt.IsDynamicBlocked);

                if (p == null || p.Count == 0)
                {
                    res.Success = false;
                    res.IsTimeout = true;
                    res.Message = $"智能体 #{tasks[i].AgentId} 找不到无冲突路径！";
                    res.ElapsedMilliseconds = sw.ElapsedMilliseconds;
                    return res;
                }

                res.AgentPaths.Add(p);
                res.TimedSchedules.Add(tp);

                int startKey = GetCellKey(tasks[i].Start.X, tasks[i].Start.Y);
                if (gridIntervals.TryGetValue(startKey, out var sList))
                {
                    for (int k = 0; k < sList.Count; k++)
                    {
                        if (sList[k].AgentId == i)
                        {
                            double actDep = (tp.Count > 1) ? tp[1].PlannedEntryTime : tp[0].PlannedExitTime;
                            sList[k] = new TimeInterval(0.0, actDep, i);
                            break;
                        }
                    }
                }

                for (int step = 0; step < tp.Count; step++)
                {
                    var pt = tp[step];
                    int cKey = GetCellKey(pt.Grid.X, pt.Grid.Y);
                    if (!gridIntervals.TryGetValue(cKey, out var cList))
                    {
                        cList = new List<TimeInterval>();
                        gridIntervals[cKey] = cList;
                    }
                    cList.Add(new TimeInterval(pt.PlannedEntryTime, pt.PlannedExitTime, i));

                    if (step < tp.Count - 1)
                    {
                        var fromPt = tp[step].Grid;
                        var toPt = tp[step + 1].Grid;
                        if (fromPt != toPt)
                        {
                            long eKey = GetUndirectedEdgeKey(fromPt, toPt);
                            if (!edgeIntervals.TryGetValue(eKey, out var eList))
                            {
                                eList = new List<EdgeInterval>();
                                edgeIntervals[eKey] = eList;
                            }
                            eList.Add(new EdgeInterval(fromPt, toPt, pt.PlannedEntryTime, tp[step + 1].PlannedExitTime, i));
                        }
                    }
                }
            }

            sw.Stop();
            res.Success = true;
            res.ElapsedMilliseconds = sw.ElapsedMilliseconds;
            string algoName = (windowSize > 0) ? "WHCA*" : "CA*";
            res.Message = $"{algoName} 成功为 {n} 台机器人规划无冲突时空路径";
            return res;
        }

        public struct EdgeInterval
        {
            public GridPoint From;
            public GridPoint To;
            public double EntryTime;
            public double ExitTime;
            public int AgentId;

            public EdgeInterval(GridPoint from, GridPoint to, double entry, double exit, int agentId)
            {
                From = from;
                To = to;
                EntryTime = entry;
                ExitTime = exit;
                AgentId = agentId;
            }

            public bool Overlaps(double otherEntry, double otherExit, double margin = 0.60)
            {
                return Math.Max(EntryTime - margin, otherEntry) < Math.Min(ExitTime + margin, otherExit);
            }
        }

        public static int GetCellKey(int x, int y) => (x << 16) | (y & 0xFFFF);

        public static long GetUndirectedEdgeKey(GridPoint a, GridPoint b)
        {
            // 获取两个相邻网格之间的无向边唯一定位键。
            // a: 边端点 A; b: 边端点 B。
            int keyA = (a.X << 16) | (a.Y & 0xFFFF);
            int keyB = (b.X << 16) | (b.Y & 0xFFFF);
            int min = Math.Min(keyA, keyB);
            int max = Math.Max(keyA, keyB);
            return ((long)min << 32) | (uint)max;
        }
    }
}
