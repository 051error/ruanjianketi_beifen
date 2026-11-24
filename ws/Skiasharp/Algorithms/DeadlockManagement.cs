using System;
using System.Collections.Generic;
using System.Drawing;
using WarehouseGridSimulation.Skia.Models;

namespace WarehouseGridSimulation.Skia.Algorithms
{
    /// <summary>
    /// 死锁与拥堵诊断状态类型。
    /// </summary>
    public enum DeadlockStatus
    {
        /// <summary>
        /// 系统状态正常，机群流动顺畅，无死锁或拥堵。
        /// </summary>
        Normal = 0,

        /// <summary>
        /// 临时队列等待（前车正在合法推进，未来资源可靠释放）。
        /// </summary>
        TemporaryWaiting = 1,

        /// <summary>
        /// 级联受阻/通道排队（排在堵塞车队后方，自身未构成环路）。
        /// </summary>
        TransitiveBlockage = 2,

        /// <summary>
        /// 检测到循环等待死锁，请求触发具备实际时间精准匹配的全局协同重规划打破僵局。
        /// </summary>
        NeedGlobalReplanning = 3,

        /// <summary>
        /// 死锁经多级应对后依然无法解脱（例如物理绝对死胡同）。
        /// </summary>
        DeadlockConfirmed = 4
    }

    /// <summary>
    /// 资源阻塞类型。
    /// </summary>
    public enum ResourceBlockType
    {
        None = 0,
        VertexLocked = 1,  // 网格顶点被其他机器人占用锁定
        EdgeLocked = 2,    // 无向边被对向机器人跨越占用
        ProximityBlocked = 3 // 处于物理车身安全距离排斥范围
    }

    /// <summary>
    /// 等待依赖图（Wait-For Graph）中的有向依赖边：WaitingAgent -> BlockingAgent
    /// </summary>
    public sealed class DependencyRelation
    {
        public int WaitingAgent { get; set; }
        public int BlockingAgent { get; set; }
        public Point ConflictGrid { get; set; }
        public ResourceBlockType BlockType { get; set; }
        public string Description { get; set; }

        public DependencyRelation(int waitingAgent, int blockingAgent, Point conflictGrid, ResourceBlockType blockType, string description)
        {
            WaitingAgent = waitingAgent;
            BlockingAgent = blockingAgent;
            ConflictGrid = conflictGrid;
            BlockType = blockType;
            Description = description;
        }

        public override string ToString()
        {
            return $"机器人 {WaitingAgent + 1} 等待 机器人 {BlockingAgent + 1} (在 {ConflictGrid}, {BlockType}): {Description}";
        }
    }

    /// <summary>
    /// 死锁循环等待环（Wait-For Cycle）。
    /// </summary>
    public sealed class DeadlockCycle
    {
        public List<int> Agents { get; } = new List<int>();
        public List<DependencyRelation> Relations { get; } = new List<DependencyRelation>();
        public double CycleDuration { get; set; }

        public string GetCycleDescription()
        {
            List<string> parts = new List<string>();
            for (int i = 0; i < Agents.Count; i++)
            {
                parts.Add((Agents[i] + 1).ToString());
            }
            if (Agents.Count > 0)
            {
                parts.Add((Agents[0] + 1).ToString());
            }
            return string.Join(" -> ", parts);
        }
    }

    /// <summary>
    /// 综合死锁诊断与应对报告。
    /// </summary>
    public sealed class DeadlockDiagnosticReport
    {
        public DeadlockStatus Status { get; set; } = DeadlockStatus.Normal;
        public List<DeadlockCycle> DeadlockCycles { get; } = new List<DeadlockCycle>();
        public List<DependencyRelation> AllDependencies { get; } = new List<DependencyRelation>();
        public Dictionary<int, string> AgentStatusNotes { get; } = new Dictionary<int, string>();
        public string ResolutionDetails { get; set; } = string.Empty;

        public bool IsDeadlockActive => Status == DeadlockStatus.DeadlockConfirmed;
    }

    /// <summary>
    /// 工业级多智能体死锁、堵塞检测与自主应对恢复系统。
    /// 周期性执行一致性快照、无进展筛选、等待依赖图构建、循环等待环搜索、状态可信核验及自适应脱困。
    /// </summary>
    public sealed class DeadlockManager
    {
        // 判定持续无进展的时间阈值（秒）
        public double StallThresholdSeconds { get; set; } = 1.6;

        // 判定死锁确认的连续持续时间（秒）
        public double DeadlockConfirmationSeconds { get; set; } = 2.2;

        private readonly double[] _lastProgressTime = new double[256];
        private readonly Point[] _lastProgressGrid = new Point[256];
        private readonly double[] _deadlockCycleStartTime = new double[256];

        public void Reset()
        {
            Array.Clear(_lastProgressTime, 0, _lastProgressTime.Length);
            Array.Clear(_lastProgressGrid, 0, _lastProgressGrid.Length);
            Array.Clear(_deadlockCycleStartTime, 0, _deadlockCycleStartTime.Length);
        }

        /// <summary>
        /// 核心周期性检测入口：获取快照、构建等待依赖图、检测环路、可信度核验并执行应对措施。
        /// </summary>
        public DeadlockDiagnosticReport DiagnoseAndResolve(
            IList<Robot> robots,
            IList<Point> targets,
            Dictionary<Point, int> lockedZones,
            Dictionary<long, int> lockedEdges,
            int gridCols,
            int gridRows,
            float cellSizeMeters,
            double currentSimSeconds)
        {
            DeadlockDiagnosticReport report = new DeadlockDiagnosticReport();
            if (robots == null || robots.Count == 0)
            {
                return report;
            }

            int n = robots.Count;

            // -------------------------------------------------------------
            // 步骤 1：获取一致的机器人、地图和资源状态快照 (Snapshot)
            // -------------------------------------------------------------
            var robotSnapshots = new List<RobotSnapshot>(n);
            for (int i = 0; i < n; i++)
            {
                Robot r = robots[i];
                Point curGrid = new Point(r.GridX, r.GridY);
                Point target = (i < targets.Count) ? targets[i] : Point.Empty;
                bool atGoal = (r.GridX == target.X && r.GridY == target.Y && r.NavState == RobotNavState.Idle);

                // 推进位置对比：若网格发生变化或处于正常平移行进中，刷新进展时间
                if (_lastProgressGrid[i] != curGrid || r.ActionState == RobotActionState.Moving)
                {
                    _lastProgressGrid[i] = curGrid;
                    _lastProgressTime[i] = currentSimSeconds;
                }

                double stallDuration = currentSimSeconds - _lastProgressTime[i];

                robotSnapshots.Add(new RobotSnapshot
                {
                    RobotId = i,
                    CurrentGrid = curGrid,
                    FinalTarget = target,
                    IsAtGoal = atGoal,
                    ActionState = r.ActionState,
                    NavState = r.NavState,
                    IsAwaitingPermission = r.IsAwaitingPermission,
                    StallDuration = stallDuration,
                    NextWantedZones = r.PeekSegmentZones()
                });
            }

            // -------------------------------------------------------------
            // 步骤 2：找出持续无有效进展的候选机器人
            // -------------------------------------------------------------
            var stalledCandidates = new HashSet<int>();
            for (int i = 0; i < n; i++)
            {
                var s = robotSnapshots[i];
                if (!s.IsAtGoal && s.IsAwaitingPermission && s.ActionState == RobotActionState.Idle)
                {
                    if (s.StallDuration >= StallThresholdSeconds)
                    {
                        stalledCandidates.Add(i);
                    }
                }
            }

            if (stalledCandidates.Count == 0)
            {
                report.Status = DeadlockStatus.Normal;
                return report;
            }

            // -------------------------------------------------------------
            // 步骤 3 & 4：分析阻塞资源和持有者，建立等待依赖图 (Wait-For Graph)
            // -------------------------------------------------------------
            // 邻接表：WaitingId -> List<DependencyRelation>
            Dictionary<int, List<DependencyRelation>> waitGraph = new Dictionary<int, List<DependencyRelation>>();
            foreach (int candidateId in stalledCandidates)
            {
                waitGraph[candidateId] = new List<DependencyRelation>();
            }

            float bodyHalf = (Robot.ChassisLengthMeters + Robot.ChassisWidthMeters) * 0.25f;
            float clearanceSq = bodyHalf * bodyHalf;

            foreach (int waitingId in stalledCandidates)
            {
                var s = robotSnapshots[waitingId];
                if (s.NextWantedZones == null || s.NextWantedZones.Count == 0)
                {
                    continue;
                }

                Point nextGrid = s.NextWantedZones[0];

                // 3.1 检查网格顶点锁持有者
                if (lockedZones.TryGetValue(nextGrid, out int zoneHolder) && zoneHolder != waitingId)
                {
                    var dep = new DependencyRelation(
                        waitingId,
                        zoneHolder,
                        nextGrid,
                        ResourceBlockType.VertexLocked,
                        $"目标栅格 {nextGrid} 被机器人 {zoneHolder + 1} 持有锁定");
                    waitGraph[waitingId].Add(dep);
                    report.AllDependencies.Add(dep);
                }

                // 3.2 检查无向跨越边持有者 (对向逆行通道占用)
                if (nextGrid != s.CurrentGrid)
                {
                    long edgeKey = GetUndirectedEdgeKey(s.CurrentGrid, nextGrid);
                    if (lockedEdges.TryGetValue(edgeKey, out int edgeHolder) && edgeHolder != waitingId)
                    {
                        var dep = new DependencyRelation(
                            waitingId,
                            edgeHolder,
                            nextGrid,
                            ResourceBlockType.EdgeLocked,
                            $"通道跨越边 ({s.CurrentGrid}->{nextGrid}) 被机器人 {edgeHolder + 1} 逆向或重叠占用");
                        waitGraph[waitingId].Add(dep);
                        report.AllDependencies.Add(dep);
                    }
                }

                // 3.3 检查物理空间排斥距离
                float cx = (nextGrid.X + 0.5f) * cellSizeMeters;
                float cy = (nextGrid.Y + 0.5f) * cellSizeMeters;
                for (int j = 0; j < n; j++)
                {
                    if (j == waitingId) continue;
                    float dx = robots[j].PositionX - cx;
                    float dy = robots[j].PositionY - cy;
                    if (dx * dx + dy * dy < clearanceSq)
                    {
                        var dep = new DependencyRelation(
                            waitingId,
                            j,
                            nextGrid,
                            ResourceBlockType.ProximityBlocked,
                            $"栅格 {nextGrid} 处于机器人 {j + 1} 物理车身接触范围内");
                        waitGraph[waitingId].Add(dep);
                        report.AllDependencies.Add(dep);
                    }
                }
            }

            // -------------------------------------------------------------
            // 步骤 5：找出图中的循环等待候选组 (Cycle Detection)
            // -------------------------------------------------------------
            List<List<int>> detectedCycles = FindCyclesInGraph(waitGraph);

            if (detectedCycles.Count == 0)
            {
                // 无循环闭环：属于队列级联等待或临时等待
                report.Status = DeadlockStatus.TransitiveBlockage;
                foreach (int cid in stalledCandidates)
                {
                    report.AgentStatusNotes[cid] = "在受控通道后方排队等待前车推进 (无环路死锁)";
                }
                return report;
            }

            // -------------------------------------------------------------
            // 步骤 6：核验候选组状态、检查可行解除动作、未来资源释放与可信度
            // -------------------------------------------------------------
            List<DeadlockCycle> confirmedDeadlocks = new List<DeadlockCycle>();

            foreach (var cycle in detectedCycles)
            {
                bool allTrulyStalled = true;
                bool futureReleaseReliable = false;

                for (int k = 0; k < cycle.Count; k++)
                {
                    int aid = cycle[k];
                    var rSnap = robotSnapshots[aid];

                    // 检查环内是否有车其实正在运动或正在原地转向
                    if (rSnap.ActionState != RobotActionState.Idle || !rSnap.IsAwaitingPermission)
                    {
                        allTrulyStalled = false;
                        futureReleaseReliable = true;
                        break;
                    }
                }

                if (!allTrulyStalled || futureReleaseReliable)
                {
                    // 数据指示环内某车即将释放资源，属于过渡等待状态
                    report.Status = DeadlockStatus.TemporaryWaiting;
                    continue;
                }

                // 环内机器人全部真实停滞：核验死锁持续时间
                int leadAgent = cycle[0];
                if (_deadlockCycleStartTime[leadAgent] <= 0.001)
                {
                    _deadlockCycleStartTime[leadAgent] = currentSimSeconds;
                }

                double deadlockDuration = currentSimSeconds - _deadlockCycleStartTime[leadAgent];

                var dlCycle = new DeadlockCycle
                {
                    CycleDuration = deadlockDuration
                };
                dlCycle.Agents.AddRange(cycle);

                // 关联各节点间依赖详情
                for (int c = 0; c < cycle.Count; c++)
                {
                    int from = cycle[c];
                    int to = cycle[(c + 1) % cycle.Count];
                    if (waitGraph.TryGetValue(from, out var list))
                    {
                        for (int d = 0; d < list.Count; d++)
                        {
                            if (list[d].BlockingAgent == to)
                            {
                                dlCycle.Relations.Add(list[d]);
                                break;
                            }
                        }
                    }
                }

                confirmedDeadlocks.Add(dlCycle);
                report.DeadlockCycles.Add(dlCycle);
            }

            if (confirmedDeadlocks.Count == 0)
            {
                report.Status = DeadlockStatus.TemporaryWaiting;
                return report;
            }

            // -------------------------------------------------------------
            // 步骤 7：智能应对与自主解脱措施 (Resolution Strategies)
            // -------------------------------------------------------------
            // 工业级核心准则：废止单车全盲脱节的局部重规划，直接触发具备全局时间基准对齐的全局协同重规划。
            // 汇总所有确认死锁环的机器人清单与阻塞详情，请求全局调度器实施在线协同重规划，从时空拓扑上彻底解冻机群。
            List<string> cycleDescriptions = new List<string>();
            foreach (var cycle in confirmedDeadlocks)
            {
                cycleDescriptions.Add($"[{cycle.GetCycleDescription()}]");
            }

            report.Status = DeadlockStatus.NeedGlobalReplanning;
            report.ResolutionDetails = $"检测到循环依赖死锁 (死锁环: {string.Join(", ", cycleDescriptions)})，触发具备实际时间精准匹配的全局协同重规划";
            return report;
        }

        /// <summary>
        /// 使用深度优先搜索（DFS）在等待依赖有向图中提取所有简单环。
        /// </summary>
        private static List<List<int>> FindCyclesInGraph(Dictionary<int, List<DependencyRelation>> graph)
        {
            List<List<int>> cycles = new List<List<int>>();
            HashSet<int> visited = new HashSet<int>();
            HashSet<int> inStack = new HashSet<int>();
            List<int> currentPath = new List<int>();

            void Dfs(int u)
            {
                visited.Add(u);
                inStack.Add(u);
                currentPath.Add(u);

                if (graph.TryGetValue(u, out var edges))
                {
                    for (int i = 0; i < edges.Count; i++)
                    {
                        int v = edges[i].BlockingAgent;
                        if (!visited.Contains(v))
                        {
                            Dfs(v);
                        }
                        else if (inStack.Contains(v))
                        {
                            // 发现环路！截取从 v 开始的闭环切片
                            int startIndex = currentPath.IndexOf(v);
                            if (startIndex >= 0)
                            {
                                List<int> cycle = new List<int>();
                                for (int k = startIndex; k < currentPath.Count; k++)
                                {
                                    cycle.Add(currentPath[k]);
                                }
                                cycles.Add(cycle);
                            }
                        }
                    }
                }

                currentPath.RemoveAt(currentPath.Count - 1);
                inStack.Remove(u);
            }

            foreach (int node in graph.Keys)
            {
                if (!visited.Contains(node))
                {
                    Dfs(node);
                }
            }

            return cycles;
        }

        private static long GetUndirectedEdgeKey(Point u, Point v)
        {
            long k1 = ((long)u.X << 16) | (uint)(u.Y & 0xFFFF);
            long k2 = ((long)v.X << 16) | (uint)(v.Y & 0xFFFF);
            if (k1 > k2) { long t = k1; k1 = k2; k2 = t; }
            return (k1 << 32) | (k2 & 0xFFFFFFFFL);
        }

        private sealed class RobotSnapshot
        {
            public int RobotId { get; set; }
            public Point CurrentGrid { get; set; }
            public Point FinalTarget { get; set; }
            public bool IsAtGoal { get; set; }
            public RobotActionState ActionState { get; set; }
            public RobotNavState NavState { get; set; }
            public bool IsAwaitingPermission { get; set; }
            public double StallDuration { get; set; }
            public List<Point> NextWantedZones { get; set; }
        }
    }
}
