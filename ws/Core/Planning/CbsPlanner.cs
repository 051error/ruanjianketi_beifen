using System;
using System.Collections.Generic;
using System.Diagnostics;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 基于冲突搜索算法 (Conflict-Based Search, CBS) 的多智能体无碰撞寻路规划器。
    /// 采用双层搜索架构，高层维护冲突约束树，底层基于时空 A* 搜索受约束的单车路径。
    /// </summary>
    public class CbsPlanner : IMultiAgentPathPlanner
    {
        public virtual string Id => "CBS";
        public virtual string Name => "CBS (基于冲突搜索)";

        public virtual MultiAgentPlanResult Plan(
            GridMap map,
            IReadOnlyList<AgentTask> tasks,
            MultiAgentPlanOptions options = null)
        {
            // 执行基于冲突搜索 (CBS) 的多智能体最优路径规划。
            // map: 仓储栅格地图; tasks: 参与调度的智能体任务清单; options: 算法配置选项。
            var opt = options ?? new MultiAgentPlanOptions();
            return ExecutePlan(map, tasks, opt, isEnhanced: false, suboptimality: 1.0);
        }

        protected MultiAgentPlanResult ExecutePlan(
            GridMap map,
            IReadOnlyList<AgentTask> tasks,
            MultiAgentPlanOptions opt,
            bool isEnhanced,
            double suboptimality)
        {
            // 执行双层冲突搜索树寻路流转。
            // opt: 规划参数选项; isEnhanced: 是否启用增强型焦点搜索 (ECBS); suboptimality: 次优松弛因子。
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

            var root = new CTNode(n);
            int initialCost = 0;

            bool IsRootTargetExclusive(int aid, GridPoint from, GridPoint to, double enterTime, double leaveTime)
            {
                for (int j = 0; j < n; j++)
                {
                    if (j != aid && tasks[j].Target == to && tasks[aid].Target != to)
                    {
                        return true;
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
                    res.Message = "CBS 初始规划超时！";
                    res.ElapsedMilliseconds = sw.ElapsedMilliseconds;
                    return res;
                }

                var (p, tp) = SpaceTimeAStar.SearchPath(
                    map,
                    tasks[i].Start,
                    tasks[i].Target,
                    i,
                    IsRootTargetExclusive,
                    SpaceTimeAStar.MaxTimeSteps,
                    suboptimality,
                    isDynamicBlocked: opt.IsDynamicBlocked);

                if (p == null || p.Count == 0)
                {
                    res.Success = false;
                    res.IsTimeout = true;
                    res.Message = $"智能体 #{tasks[i].AgentId} 独立无解！";
                    res.ElapsedMilliseconds = sw.ElapsedMilliseconds;
                    return res;
                }

                root.Paths.Add(p);
                root.TimedPaths.Add(tp);
                initialCost += p.Count;
            }

            root.TotalCost = initialCost;
            root.ConflictCount = CountIntervalConflicts(root.TimedPaths);

            var openTree = new PriorityQueue<CTNode, double>();
            openTree.Enqueue(root, isEnhanced ? (root.TotalCost + root.ConflictCount * 5.0) : root.TotalCost);

            int expandedNodes = 0;
            const int maxExpandedNodes = 400;

            while (openTree.Count > 0)
            {
                expandedNodes++;

                if (sw.ElapsedMilliseconds > opt.TimeoutMilliseconds || expandedNodes > maxExpandedNodes)
                {
                    // 若冲突树过于庞大或超时，降级至时空预订协同 A* 快速保底
                    if (isEnhanced || tasks.Count >= 15)
                    {
                        var caFallback = new CAStarPlanner();
                        return caFallback.Plan(map, tasks, opt);
                    }

                    res.Success = false;
                    res.IsTimeout = true;
                    res.Message = "CBS 调度超时：冲突树组合爆炸，任务失败！";
                    res.ElapsedMilliseconds = sw.ElapsedMilliseconds;
                    return res;
                }

                var node = openTree.Dequeue();

                if (!FindNextIntervalConflict(node.TimedPaths, out Conflict conflict))
                {
                    sw.Stop();
                    res.Success = true;
                    res.AgentPaths = node.Paths;
                    res.TimedSchedules = node.TimedPaths;
                    res.ElapsedMilliseconds = sw.ElapsedMilliseconds;
                    string name = isEnhanced ? "ECBS" : "CBS";
                    res.Message = $"{name} 成功规划 {n} 条无冲突时空路径 (展开 {expandedNodes} 节点)";
                    return res;
                }

                for (int b = 0; b < 2; b++)
                {
                    int agentIndex = (b == 0) ? conflict.AgentA : conflict.AgentB;
                    var child = node.Clone();

                    double cStart = conflict.IsEdgeConflict
                        ? (b == 0 ? conflict.AgentBStart : conflict.AgentAStart)
                        : conflict.OverlapStart;
                    double cEnd = conflict.IsEdgeConflict
                        ? (b == 0 ? conflict.AgentBEnd : conflict.AgentAEnd)
                        : conflict.OverlapEnd;

                    var newC = new CBSConstraint
                    {
                        AgentId = agentIndex,
                        IsEdge = conflict.IsEdgeConflict,
                        Loc = conflict.IsEdgeConflict ? (b == 0 ? conflict.Loc : conflict.LocTo) : conflict.Loc,
                        LocTo = conflict.IsEdgeConflict ? (b == 0 ? conflict.LocTo : conflict.Loc) : GridPoint.Empty,
                        StartTime = cStart,
                        EndTime = cEnd
                    };
                    child.Constraints.Add(newC);

                    bool IsViolated(int aid, GridPoint from, GridPoint to, double enterTime, double leaveTime)
                    {
                        if (aid != agentIndex) return false;

                        for (int j = 0; j < n; j++)
                        {
                            if (j != aid && tasks[j].Target == to && tasks[aid].Target != to)
                            {
                                return true;
                            }
                        }

                        for (int k = 0; k < child.Constraints.Count; k++)
                        {
                            var c = child.Constraints[k];
                            if (c.AgentId != agentIndex) continue;

                            if (c.IsEdge)
                            {
                                if (from == c.Loc && to == c.LocTo)
                                {
                                    if (Math.Max(enterTime, c.StartTime) < Math.Min(leaveTime, c.EndTime))
                                    {
                                        return true;
                                    }
                                }
                            }
                            else
                            {
                                if (to == c.Loc)
                                {
                                    if (Math.Max(enterTime, c.StartTime) < Math.Min(leaveTime, c.EndTime))
                                    {
                                        return true;
                                    }
                                }
                            }
                        }
                        return false;
                    }

                    var (newP, newTP) = SpaceTimeAStar.SearchPath(
                        map,
                        tasks[agentIndex].Start,
                        tasks[agentIndex].Target,
                        agentIndex,
                        IsViolated,
                        SpaceTimeAStar.MaxTimeSteps,
                        suboptimality,
                        isDynamicBlocked: opt.IsDynamicBlocked);

                    if (newP != null && newP.Count > 0)
                    {
                        child.Paths[agentIndex] = newP;
                        child.TimedPaths[agentIndex] = newTP;
                        child.TotalCost = 0;
                        for (int p = 0; p < child.Paths.Count; p++)
                        {
                            child.TotalCost += child.Paths[p].Count;
                        }
                        child.ConflictCount = CountIntervalConflicts(child.TimedPaths);

                        double priority = isEnhanced
                            ? (child.TotalCost + child.ConflictCount * 5.0)
                            : child.TotalCost;
                        openTree.Enqueue(child, priority);
                    }
                }
            }

            sw.Stop();
            res.Success = false;
            res.IsTimeout = true;
            res.Message = "多机调度无可行解 (死锁)！";
            res.ElapsedMilliseconds = sw.ElapsedMilliseconds;
            return res;
        }

        protected struct CBSConstraint
        {
            public int AgentId;
            public bool IsEdge;
            public GridPoint Loc;
            public GridPoint LocTo;
            public double StartTime;
            public double EndTime;
        }

        protected struct Conflict
        {
            public int AgentA;
            public int AgentB;
            public bool IsEdgeConflict;
            public GridPoint Loc;
            public GridPoint LocTo;
            public double OverlapStart;
            public double OverlapEnd;
            public double AgentAStart;
            public double AgentAEnd;
            public double AgentBStart;
            public double AgentBEnd;
        }

        protected sealed class CTNode
        {
            public List<List<GridPoint>> Paths;
            public List<List<TimedPathPoint>> TimedPaths;
            public List<CBSConstraint> Constraints;
            public int TotalCost;
            public int ConflictCount;

            public CTNode(int n)
            {
                Paths = new List<List<GridPoint>>(n);
                TimedPaths = new List<List<TimedPathPoint>>(n);
                Constraints = new List<CBSConstraint>();
            }

            public CTNode Clone()
            {
                var copy = new CTNode(Paths.Count)
                {
                    Paths = new List<List<GridPoint>>(Paths),
                    TimedPaths = new List<List<TimedPathPoint>>(TimedPaths),
                    Constraints = new List<CBSConstraint>(Constraints),
                    TotalCost = TotalCost,
                    ConflictCount = ConflictCount
                };
                return copy;
            }
        }

        protected static bool FindNextIntervalConflict(List<List<TimedPathPoint>> schedules, out Conflict conflict)
        {
            // 扫描时空轨迹集合并定位发生的最早顶点或边冲突。
            // schedules: 所有智能体的时空轨迹列表; conflict: 检测到的最早冲突结构体。
            conflict = default;
            bool found = false;
            double minOverlapStart = double.MaxValue;

            for (int i = 0; i < schedules.Count; i++)
            {
                var schedI = schedules[i];
                for (int j = i + 1; j < schedules.Count; j++)
                {
                    var schedJ = schedules[j];

                    for (int pi = 0; pi < schedI.Count; pi++)
                    {
                        var ptI = schedI[pi];
                        for (int pj = 0; pj < schedJ.Count; pj++)
                        {
                            var ptJ = schedJ[pj];
                            if (ptI.Grid == ptJ.Grid)
                            {
                                double overlapStart = Math.Max(ptI.PlannedEntryTime, ptJ.PlannedEntryTime);
                                double overlapEnd = Math.Min(ptI.PlannedExitTime, ptJ.PlannedExitTime);
                                if (overlapStart < overlapEnd)
                                {
                                    if (overlapStart < minOverlapStart)
                                    {
                                        minOverlapStart = overlapStart;
                                        conflict = new Conflict
                                        {
                                            AgentA = i,
                                            AgentB = j,
                                            IsEdgeConflict = false,
                                            Loc = ptI.Grid,
                                            LocTo = GridPoint.Empty,
                                            OverlapStart = overlapStart,
                                            OverlapEnd = overlapEnd
                                        };
                                        found = true;
                                    }
                                }
                            }
                        }
                    }

                    for (int pi = 0; pi < schedI.Count - 1; pi++)
                    {
                        var uI = schedI[pi].Grid;
                        var vI = schedI[pi + 1].Grid;
                        if (uI == vI) continue;

                        for (int pj = 0; pj < schedJ.Count - 1; pj++)
                        {
                            var uJ = schedJ[pj].Grid;
                            var vJ = schedJ[pj + 1].Grid;
                            if (uJ == vJ) continue;

                            if (uI == vJ && vI == uJ)
                            {
                                double startI = schedI[pi].PlannedEntryTime;
                                double exitI = Math.Min(schedI[pi + 1].PlannedExitTime, schedI[pi + 1].PlannedEntryTime + SpaceTimeAStar.StepPhysicalDuration + SpaceTimeAStar.ConservativeBuffer);
                                double startJ = schedJ[pj].PlannedEntryTime;
                                double exitJ = Math.Min(schedJ[pj + 1].PlannedExitTime, schedJ[pj + 1].PlannedEntryTime + SpaceTimeAStar.StepPhysicalDuration + SpaceTimeAStar.ConservativeBuffer);

                                const double edgeDriftMargin = 2.0;
                                bool isTemporalOverlap = Math.Max(startI - edgeDriftMargin, startJ) < Math.Min(exitI + edgeDriftMargin, exitJ);
                                bool isStepSync = Math.Abs(pi - pj) <= 3 && Math.Abs(startI - startJ) < 3.0;

                                if (isTemporalOverlap || isStepSync)
                                {
                                    double overlapStart = Math.Max(startI, startJ);
                                    double overlapEnd = Math.Min(exitI, exitJ);
                                    if (overlapStart >= overlapEnd)
                                    {
                                        overlapStart = Math.Min(startI, startJ);
                                        overlapEnd = Math.Max(exitI, exitJ);
                                    }

                                    if (overlapStart < minOverlapStart)
                                    {
                                        minOverlapStart = overlapStart;
                                        conflict = new Conflict
                                        {
                                            AgentA = i,
                                            AgentB = j,
                                            IsEdgeConflict = true,
                                            Loc = uI,
                                            LocTo = vI,
                                            OverlapStart = overlapStart,
                                            OverlapEnd = overlapEnd,
                                            AgentAStart = startI,
                                            AgentAEnd = exitI,
                                            AgentBStart = startJ,
                                            AgentBEnd = exitJ
                                        };
                                        found = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            return found;
        }

        protected static int CountIntervalConflicts(List<List<TimedPathPoint>> schedules)
        {
            // 统计给定轨迹集合中的时空冲突总数作为启发式代价。
            int count = 0;
            for (int i = 0; i < schedules.Count; i++)
            {
                var schedI = schedules[i];
                for (int j = i + 1; j < schedules.Count; j++)
                {
                    var schedJ = schedules[j];
                    for (int pi = 0; pi < schedI.Count; pi++)
                    {
                        var ptI = schedI[pi];
                        for (int pj = 0; pj < schedJ.Count; pj++)
                        {
                            var ptJ = schedJ[pj];
                            if (ptI.Grid == ptJ.Grid)
                            {
                                if (Math.Max(ptI.PlannedEntryTime, ptJ.PlannedEntryTime) < Math.Min(ptI.PlannedExitTime, ptJ.PlannedExitTime))
                                {
                                    count++;
                                }
                            }
                        }
                    }

                    for (int pi = 0; pi < schedI.Count - 1; pi++)
                    {
                        var uI = schedI[pi].Grid;
                        var vI = schedI[pi + 1].Grid;
                        if (uI == vI) continue;

                        for (int pj = 0; pj < schedJ.Count - 1; pj++)
                        {
                            var uJ = schedJ[pj].Grid;
                            var vJ = schedJ[pj + 1].Grid;
                            if (uJ == vJ) continue;

                            if (uI == vJ && vI == uJ)
                            {
                                count++;
                            }
                        }
                    }
                }
            }
            return count;
        }
    }
}
