using System;
using System.Collections.Generic;
using System.Linq;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;
using WarehouseGridSimulation.Core.Motion;
using WarehouseGridSimulation.Core.Robots;

namespace WarehouseGridSimulation.Core.Traffic
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
        /// 检测到循环等待死锁，需触发全局在线时空协同重规划打破僵局。
        /// </summary>
        NeedGlobalReplanning = 3,

        /// <summary>
        /// 死锁经多级应对后依然无法解脱（例如极端物理死胡同堵塞）。
        /// </summary>
        DeadlockConfirmed = 4,

        /// <summary>
        /// 静态车位/到站停泊车阻挡（前车已完成任务停稳，需在途车自主动态绕障重规划）。
        /// </summary>
        StationaryBlockage = 5,

        /// <summary>
        /// 级联长等待/饿死受阻（等待前车或前方死锁区域过久超过超时阈值，需触发应急自主绕障重规划）。
        /// </summary>
        CascadingLongWaitBlockage = 6
    }

    /// <summary>
    /// 资源阻塞类型。
    /// </summary>
    public enum ResourceBlockType
    {
        None = 0,
        VertexLocked = 1, // 网格顶点被其他机器人占用锁定
        EdgeLocked = 2    // 通道无向边被其他机器人对向或重叠占用
    }

    /// <summary>
    /// 等待依赖图 (Wait-For Graph) 中的有向依赖边：WaitingAgent -> BlockingAgent。
    /// </summary>
    public sealed class DependencyRelation
    {
        public int WaitingAgent { get; }
        public int BlockingAgent { get; }
        public GridPoint ConflictGrid { get; }
        public ResourceBlockType BlockType { get; }
        public string Description { get; }

        public DependencyRelation(int waitingAgent, int blockingAgent, GridPoint conflictGrid, ResourceBlockType blockType, string description)
        {
            WaitingAgent = waitingAgent;
            BlockingAgent = blockingAgent;
            ConflictGrid = conflictGrid;
            BlockType = blockType;
            Description = description;
        }

        public override string ToString() => $"机器人 #{WaitingAgent} 等待 机器人 #{BlockingAgent} ({ConflictGrid}, {BlockType}): {Description}";
    }

    /// <summary>
    /// 死锁循环等待环 (Wait-For Cycle)。
    /// </summary>
    public sealed class DeadlockCycle
    {
        public List<int> Agents { get; } = new List<int>();
        public List<DependencyRelation> Relations { get; } = new List<DependencyRelation>();
        public double CycleDuration { get; set; }
        public int BestVictimAgentId { get; set; } = -1;

        public string GetCycleDescription()
        {
            var parts = Agents.Select(a => $"#{a}").ToList();
            if (parts.Count > 0)
            {
                parts.Add($"#{Agents[0]}");
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
        public List<int> StationaryBlockedAgents { get; } = new List<int>();
        public List<int> LongWaitBlockedAgents { get; } = new List<int>();
        public string ResolutionDetails { get; set; } = string.Empty;

        public bool IsDeadlockActive => Status == DeadlockStatus.NeedGlobalReplanning ||
                                        Status == DeadlockStatus.DeadlockConfirmed ||
                                        Status == DeadlockStatus.StationaryBlockage ||
                                        Status == DeadlockStatus.CascadingLongWaitBlockage;
    }

    /// <summary>
    /// 工业级多智能体死锁检测与自主应对管理系统。
    /// 周期性构建等待依赖图 (Wait-For Graph) 并执行 DFS 环路搜索，提供可靠死锁识别与脱困决策。
    /// </summary>
    public sealed class DeadlockManager
    {
        public double StallThresholdSeconds { get; set; } = 0.6;
        public double DeadlockConfirmationSeconds { get; set; } = 0.4;
        public double LongWaitThresholdSeconds { get; set; } = 3.5;

        private readonly Dictionary<int, double> _lastProgressTime = new Dictionary<int, double>();
        private readonly Dictionary<int, GridPoint> _lastProgressGrid = new Dictionary<int, GridPoint>();
        private readonly Dictionary<int, double> _deadlockCycleStartTime = new Dictionary<int, double>();

        public void Reset()
        {
            // 重置死锁诊断器内部所有时间戳与跟踪记录。
            _lastProgressTime.Clear();
            _lastProgressGrid.Clear();
            _deadlockCycleStartTime.Clear();
        }

        public DeadlockDiagnosticReport Diagnose(
            IReadOnlyList<SimulatedRobot> robots,
            GridReservationManager reservations,
            GridMap map,
            float currentSimSeconds,
            Func<int, MotionController> getController)
        {
            // 周期性执行等待依赖图构建与循环死锁检测。
            // robots: 当前仿真机群列表; reservations: 交通资源占用管理器; map: 仓储栅格地图; currentSimSeconds: 当前仿真时间秒数; getController: 运动控制器提取委托。
            var report = new DeadlockDiagnosticReport();
            if (robots == null || robots.Count == 0 || reservations == null)
            {
                return report;
            }

            // 1. 停滞候选车筛选 (对比网格位移与运动状态)
            var stalledCandidates = new List<int>();
            foreach (var r in robots)
            {
                var controller = getController?.Invoke(r.Id);
                bool hasActiveMission = controller != null && controller.HasPath && !controller.IsPathCompleted;

                if (!hasActiveMission)
                {
                    _lastProgressGrid[r.Id] = r.ConfirmedGrid;
                    _lastProgressTime[r.Id] = currentSimSeconds;
                    continue;
                }

                // 排除合法旋转与运动状态：处于旋转模式、具有显著角速度、具有线速度或发生栅格位移，均非死锁停滞
                bool isRotating = r.MotionState == RobotMotionState.TurningLeft ||
                                  r.MotionState == RobotMotionState.TurningRight ||
                                  Math.Abs(r.AngularVelocity) > 0.05f;

                if (isRotating || !_lastProgressGrid.TryGetValue(r.Id, out var lastGrid) || lastGrid != r.ConfirmedGrid || r.LinearVelocity > 0.05f)
                {
                    _lastProgressGrid[r.Id] = r.ConfirmedGrid;
                    _lastProgressTime[r.Id] = currentSimSeconds;
                }

                double stallDuration = currentSimSeconds - (_lastProgressTime.TryGetValue(r.Id, out double t) ? t : currentSimSeconds);
                if (stallDuration >= StallThresholdSeconds)
                {
                    stalledCandidates.Add(r.Id);
                }
            }

            if (stalledCandidates.Count == 0)
            {
                report.Status = DeadlockStatus.Normal;
                return report;
            }

            // 2. 分析受阻资源与持有者，构建有向等待依赖图 (Wait-For Graph)
            var waitGraph = new Dictionary<int, List<DependencyRelation>>();
            var stationaryBlockedCandidates = new List<int>();

            bool IsRobotStationaryParked(int rId)
            {
                var rBot = robots.FirstOrDefault(b => b.Id == rId);
                var rCtrl = getController?.Invoke(rId);
                return rBot != null && rCtrl != null &&
                       (rCtrl.IsPathCompleted || rCtrl.CurrentPath.Count == 0) &&
                       rBot.LinearVelocity <= 0.05f;
            }

            foreach (int candidateId in stalledCandidates)
            {
                waitGraph[candidateId] = new List<DependencyRelation>();
            }

            foreach (int waitingId in stalledCandidates)
            {
                var controller = getController?.Invoke(waitingId);
                if (controller == null || !controller.HasPath) continue;

                // 查找该小车下一步尝试通行但尚未获准的网格
                int nextIndex = controller.AuthorizedIndex + 1;
                if (nextIndex >= controller.CurrentPath.Count) continue;

                var nextGrid = controller.CurrentPath[nextIndex];
                var fromGrid = controller.CurrentPath[controller.AuthorizedIndex];

                // 2.1 检查目标栅格锁持有者
                if (reservations.OccupiedGrids.TryGetValue(nextGrid, out int occHolder) && occHolder != waitingId)
                {
                    var dep = new DependencyRelation(waitingId, occHolder, nextGrid, ResourceBlockType.VertexLocked, $"目标栅格 {nextGrid} 被机器人 #{occHolder} 物理占用");
                    waitGraph[waitingId].Add(dep);
                    report.AllDependencies.Add(dep);

                    if (IsRobotStationaryParked(occHolder))
                    {
                        stationaryBlockedCandidates.Add(waitingId);
                    }
                }
                else if (reservations.ReservedGrids.TryGetValue(nextGrid, out int resHolder) && resHolder != waitingId)
                {
                    var dep = new DependencyRelation(waitingId, resHolder, nextGrid, ResourceBlockType.VertexLocked, $"目标栅格 {nextGrid} 被机器人 #{resHolder} 优先预约");
                    waitGraph[waitingId].Add(dep);
                    report.AllDependencies.Add(dep);

                    if (IsRobotStationaryParked(resHolder))
                    {
                        stationaryBlockedCandidates.Add(waitingId);
                    }
                }

                // 2.2 检查通道边持有者 (对向通道逆穿占用)
                if (fromGrid != nextGrid)
                {
                    long revKey = GridReservationManager.GetDirectedEdgeKey(nextGrid, fromGrid);
                    long unKey = GridReservationManager.GetUndirectedEdgeKey(fromGrid, nextGrid);
                    int edgeHolder = -1;
                    if (reservations.ReservedEdges.TryGetValue(revKey, out int h1) && h1 != waitingId) edgeHolder = h1;
                    else if (reservations.ReservedEdges.TryGetValue(unKey, out int h2) && h2 != waitingId) edgeHolder = h2;

                    if (edgeHolder >= 0)
                    {
                        var dep = new DependencyRelation(waitingId, edgeHolder, nextGrid, ResourceBlockType.EdgeLocked, $"通道边 ({fromGrid}<->{nextGrid}) 被机器人 #{edgeHolder} 逆向或重叠占用");
                        waitGraph[waitingId].Add(dep);
                        report.AllDependencies.Add(dep);
                    }
                }
            }

            // 3. 基于 DFS 检测依赖图中的闭环等待环路
            var detectedCycles = FindCyclesInGraph(waitGraph);
            if (detectedCycles.Count == 0)
            {
                // 无环路：检查是否存在在途小车被已到站停泊车永久阻挡的单向阻塞
                if (stationaryBlockedCandidates.Count > 0)
                {
                    report.Status = DeadlockStatus.StationaryBlockage;
                    report.StationaryBlockedAgents.AddRange(stationaryBlockedCandidates.Distinct());
                    var blockedDesc = string.Join(", ", report.StationaryBlockedAgents.Select(id => $"#{id}"));
                    report.ResolutionDetails = $"在途机器人 [{blockedDesc}] 前方被已到站停泊车永久阻挡，请求触发单车自主动态绕障重规划";
                    return report;
                }

                // 无环路：检查是否存在受阻等待时间超过阈值的长等待车辆
                var longWaitCandidates = new List<int>();
                foreach (int stalledId in stalledCandidates)
                {
                    float waitSec = reservations.GetRobotWaitTime(stalledId);
                    double stallDuration = currentSimSeconds - (_lastProgressTime.TryGetValue(stalledId, out double pt) ? pt : currentSimSeconds);
                    if (waitSec >= (float)LongWaitThresholdSeconds || stallDuration >= LongWaitThresholdSeconds)
                    {
                        longWaitCandidates.Add(stalledId);
                    }
                }

                if (longWaitCandidates.Count > 0)
                {
                    report.Status = DeadlockStatus.CascadingLongWaitBlockage;
                    report.LongWaitBlockedAgents.AddRange(longWaitCandidates.Distinct());
                    var blockedDesc = string.Join(", ", report.LongWaitBlockedAgents.Select(id => $"#{id}"));
                    report.ResolutionDetails = $"在途机器人 [{blockedDesc}] 等待资源超过 {LongWaitThresholdSeconds:F1}s，请求触发单车自主动态绕障重规划";
                    return report;
                }

                // 无环路且前车在移动且未超时：属于正常通道排队等待前车释放
                report.Status = DeadlockStatus.TransitiveBlockage;
                report.ResolutionDetails = "机群处于正常队列排队等待状态，无环路死锁";
                return report;
            }

            // 4. 真实死锁核验与持续时间确认
            var confirmedDeadlocks = new List<DeadlockCycle>();
            foreach (var cycle in detectedCycles)
            {
                int leadAgent = cycle[0];
                if (!_deadlockCycleStartTime.TryGetValue(leadAgent, out double startTime) || startTime <= 0.001)
                {
                    startTime = currentSimSeconds;
                    _deadlockCycleStartTime[leadAgent] = startTime;
                }

                double deadlockDuration = currentSimSeconds - startTime;
                if (deadlockDuration >= DeadlockConfirmationSeconds)
                {
                    var dlCycle = new DeadlockCycle { CycleDuration = deadlockDuration };
                    dlCycle.Agents.AddRange(cycle);

                    for (int c = 0; c < cycle.Count; c++)
                    {
                        int from = cycle[c];
                        int to = cycle[(c + 1) % cycle.Count];
                        if (waitGraph.TryGetValue(from, out var list))
                        {
                            var matchedDep = list.FirstOrDefault(d => d.BlockingAgent == to);
                            if (matchedDep != null) dlCycle.Relations.Add(matchedDep);
                        }
                    }

                    dlCycle.BestVictimAgentId = EvaluateBestVictim(dlCycle, robots, map, reservations, getController);

                    confirmedDeadlocks.Add(dlCycle);
                    report.DeadlockCycles.Add(dlCycle);
                }
            }

            if (confirmedDeadlocks.Count > 0)
            {
                var descList = confirmedDeadlocks.Select(c => $"[{c.GetCycleDescription()} (外围破环车 #{c.BestVictimAgentId})]");
                report.Status = DeadlockStatus.NeedGlobalReplanning;
                report.ResolutionDetails = $"检测到死锁环: {string.Join(", ", descList)}，请求触发外围优先重规划自主脱困";
            }
            else
            {
                report.Status = DeadlockStatus.TemporaryWaiting;
            }

            return report;
        }

        private static int EvaluateBestVictim(
            DeadlockCycle cycle,
            IReadOnlyList<SimulatedRobot> robots,
            GridMap map,
            GridReservationManager reservations,
            Func<int, MotionController> getController)
        {
            // 在死锁环中评估并挑选最易于侧向逃逸且对全局影响最小的外围破环智能体。
            // cycle: 死锁循环等待环; getController: 运动控制器提取委托。
            int bestAgentId = cycle.Agents.FirstOrDefault();
            double bestScore = double.MinValue;

            (int X, int Y)[] orthogonalNeighbors = new (int, int)[]
            {
                (0, -1), (1, 0), (0, 1), (-1, 0)
            };

            foreach (int aid in cycle.Agents)
            {
                var r = robots.FirstOrDefault(rb => rb.Id == aid);
                var mc = getController?.Invoke(aid);
                if (r == null || mc == null) continue;

                int freeNeighbors = 0;
                foreach (var dir in orthogonalNeighbors)
                {
                    var neighbor = new GridPoint(r.ConfirmedGrid.X + dir.X, r.ConfirmedGrid.Y + dir.Y);
                    if (map != null && map.IsWalkable(neighbor))
                    {
                        bool isOccupied = reservations.OccupiedGrids.ContainsKey(neighbor);
                        bool isReserved = reservations.ReservedGrids.ContainsKey(neighbor);
                        if (!isOccupied && !isReserved)
                        {
                            freeNeighbors++;
                        }
                    }
                }

                int remainingSteps = mc.HasPath ? Math.Max(0, mc.CurrentPath.Count - mc.CurrentTargetIndex) : 0;
                double score = freeNeighbors * 100.0 + remainingSteps * 1.0;

                if (score > bestScore)
                {
                    bestScore = score;
                    bestAgentId = aid;
                }
            }

            return bestAgentId;
        }

        private static List<List<int>> FindCyclesInGraph(Dictionary<int, List<DependencyRelation>> graph)
        {
            // 使用深度优先搜索 (DFS) 在等待依赖图中提取简单环。
            // graph: 等待依赖有向图邻接表。
            var cycles = new List<List<int>>();
            var visited = new HashSet<int>();
            var inStack = new HashSet<int>();
            var currentPath = new List<int>();

            void Dfs(int u)
            {
                visited.Add(u);
                inStack.Add(u);
                currentPath.Add(u);

                if (graph.TryGetValue(u, out var edges))
                {
                    foreach (var edge in edges)
                    {
                        int v = edge.BlockingAgent;
                        if (!visited.Contains(v))
                        {
                            Dfs(v);
                        }
                        else if (inStack.Contains(v))
                        {
                            int cycleStartIdx = currentPath.IndexOf(v);
                            if (cycleStartIdx >= 0)
                            {
                                var cycle = currentPath.Skip(cycleStartIdx).ToList();
                                if (cycle.Count >= 2)
                                {
                                    cycles.Add(cycle);
                                }
                            }
                        }
                    }
                }

                currentPath.RemoveAt(currentPath.Count - 1);
                inStack.Remove(u);
            }

            foreach (var node in graph.Keys)
            {
                if (!visited.Contains(node))
                {
                    Dfs(node);
                }
            }

            return cycles;
        }
    }
}
