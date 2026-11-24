using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;

namespace WarehouseGridSimulation.Skia.Algorithms
{
    /// <summary>
    /// 多智能体路径规划调度算法枚举。
    /// </summary>
    public enum MultiAgentAlgorithm
    {
        /// <summary>
        /// 基于冲突搜索算法 (Conflict-Based Search)，精确双层多智能体无碰撞寻路算法。
        /// </summary>
        CBS = 0,

        /// <summary>
        /// 增强型基于冲突搜索算法 (Enhanced CBS)，通过次优焦点搜索快速消解大规模冲突。
        /// </summary>
        ECBS = 1,

        /// <summary>
        /// 协同 A* 算法 (Cooperative A*)，基于全局时空预订表的快速序列调度算法。
        /// </summary>
        CAStar = 2,

        /// <summary>
        /// 基于时间窗口的分层协同 A* 算法 (Windowed Hierarchical CA*)，限制时空窗口以支持大规模实时调度。
        /// </summary>
        WHCAStar = 3
    }

    /// <summary>
    /// 单个机器人的调度任务定义（包含智能体编号、起点与目标点）。
    /// </summary>
    public struct AgentTask
    {
        public int AgentId;
        public Point Start;
        public Point Target;

        public AgentTask(int agentId, Point start, Point target)
        {
            AgentId = agentId;
            Start = start;
            Target = target;
        }
    }

    /// <summary>
    /// 包含连续保守时间区间的预约路径节点。
    /// </summary>
    public struct TimedPathPoint
    {
        public Point Grid;
        public double PlannedEntryTime; // 计划进入该网格的开始时刻 (秒)
        public double PlannedExitTime;  // 计划完全离开该网格的时刻 (秒，含保守安全缓冲)

        public TimedPathPoint(Point grid, double entryTime, double exitTime)
        {
            Grid = grid;
            PlannedEntryTime = entryTime;
            PlannedExitTime = exitTime;
        }
    }

    /// <summary>
    /// 保守占用时间区间定义。
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
    /// 多智能体调度规划输出结果。
    /// </summary>
    public sealed class MultiAgentPlanResult
    {
        public bool Success { get; set; }
        public bool IsTimeout { get; set; }
        public List<List<Point>> AgentPaths { get; set; }
        public List<List<TimedPathPoint>> TimedSchedules { get; set; }
        public string Message { get; set; }
        public long ElapsedMilliseconds { get; set; }

        public MultiAgentPlanResult()
        {
            AgentPaths = new List<List<Point>>();
            TimedSchedules = new List<List<TimedPathPoint>>();
            Message = string.Empty;
        }
    }

    /// <summary>
    /// 统一底层时空 A* 算法规划器 (Space-Time Interval A*)。
    /// 将无厚度的离散时刻扩展为保守的连续占用时间区间 [t_entry, t_exit + buffer]，
    /// 统一为 CBS、ECBS、CA*、WHCA* 提供无冗余的底层时空 A* 搜索核心。
    /// </summary>
    public static class SpaceTimeAStar
    {
        public const int MaxTimeSteps = 160;
        public const double TurnPenaltyCost = 2.5;

        // 默认动力学物理步长换算常数 (米/秒)
        public const double DefaultSpeed = 1.5;
        public const double CellSize = 0.55;
        public const double StepPhysicalDuration = CellSize / DefaultSpeed; // 约 0.367 秒/格
        public const double TurnPhysicalDuration = 0.50;                     // 原地旋转约 0.50 秒
        public const double ConservativeBuffer = 0.25;                      // 保守安全时间裕量 0.25 秒

        private static readonly Point[] DirectionsWithWait = new Point[]
        {
            new Point(0, 0),  // 原地等待
            new Point(0, -1), // 上
            new Point(1, 0),  // 右
            new Point(0, 1),  // 下
            new Point(-1, 0)  // 左
        };

        /// <summary>
        /// 执行带保守时间区间检验的时空 A* 寻路。
        /// </summary>
        public static (List<Point> Path, List<TimedPathPoint> TimedPath) SearchPath(
            Point start,
            Point target,
            int gridCols,
            int gridRows,
            int agentId,
            Func<int, Point, Point, double, double, bool> isIntervalConflict,
            int maxSteps = MaxTimeSteps,
            double suboptimality = 1.0)
        {
            if (start == target)
            {
                var p = new List<Point> { start };
                var tp = new List<TimedPathPoint> { new TimedPathPoint(start, 0.0, 1000.0) };
                return (p, tp);
            }

            PriorityQueue<STNode, double> openSet = new PriorityQueue<STNode, double>();
            Dictionary<int, double> bestCostAtState = new Dictionary<int, double>();

            double initialH = CalculateHeuristic(start, target, Point.Empty, suboptimality);
            STNode startNode = new STNode(start.X, start.Y, 0, 0.0, Point.Empty, 0.0, initialH);
            openSet.Enqueue(startNode, startNode.FCost);
            bestCostAtState[GetStateKey(start.X, start.Y, 0)] = 0.0;

            STNode bestGoalNode = null;

            while (openSet.Count > 0)
            {
                STNode current = openSet.Dequeue();

                // 达到目标网格判定
                if (current.X == target.X && current.Y == target.Y)
                {
                    bool tailClear = true;
                    double goalLeaveTime = current.PhysTime + 1000.0; // 到达终点后持续保守锁定
                    if (isIntervalConflict != null && isIntervalConflict(agentId, target, target, current.PhysTime, goalLeaveTime))
                    {
                        tailClear = false;
                    }

                    if (tailClear)
                    {
                        bestGoalNode = current;
                        break;
                    }
                }

                if (current.Time >= maxSteps)
                {
                    continue;
                }

                int nextStep = current.Time + 1;
                Point currPt = new Point(current.X, current.Y);

                for (int d = 0; d < DirectionsWithWait.Length; d++)
                {
                    int nx = current.X + DirectionsWithWait[d].X;
                    int ny = current.Y + DirectionsWithWait[d].Y;

                    if (nx < 0 || nx >= gridCols || ny < 0 || ny >= gridRows)
                    {
                        continue;
                    }

                    // 严禁 180° 倒车掉头摆动：动作不得退回刚离开的父节点网格，避让必须通过原地等待或侧向绕行实现
                    if (current.Parent != null && d != 0 && nx == current.Parent.X && ny == current.Parent.Y)
                    {
                        continue;
                    }

                    Point nextPt = new Point(nx, ny);
                    Point outDir = DirectionsWithWait[d];

                    // 计算该动作需要的物理耗时区间 (考虑转弯与直行)
                    double actionDuration;
                    double stepCost;

                    if (d == 0)
                    {
                        // 原地等待避让让行
                        actionDuration = StepPhysicalDuration;
                        stepCost = 1.05;
                    }
                    else
                    {
                        actionDuration = StepPhysicalDuration;
                        stepCost = 1.0;

                        // 转向惩罚判定：若与上一动作方向改变，计入转向耗时与转向代价
                        if (current.InDir != Point.Empty && current.InDir != outDir)
                        {
                            actionDuration += TurnPhysicalDuration;
                            stepCost += TurnPenaltyCost;
                        }
                    }

                    double enterTime = current.PhysTime;
                    double leaveTime = enterTime + actionDuration + ConservativeBuffer; // 包含保守时间裕量

                    // 核心校验：区间冲突判定 [enterTime, leaveTime] 是否与其它智能体的时间预约重叠
                    if (isIntervalConflict != null && isIntervalConflict(agentId, currPt, nextPt, enterTime, leaveTime))
                    {
                        continue;
                    }

                    double newG = current.GCost + stepCost;
                    int stateKey = GetStateKey(nx, ny, nextStep);

                    if (bestCostAtState.TryGetValue(stateKey, out double existingG) && newG >= existingG)
                    {
                        continue;
                    }

                    bestCostAtState[stateKey] = newG;
                    Point nextInDir = (d == 0) ? current.InDir : outDir;
                    double nextPhysTime = current.PhysTime + actionDuration;
                    double h = CalculateHeuristic(nextPt, target, nextInDir, suboptimality);

                    STNode nextNode = new STNode(nx, ny, nextStep, nextPhysTime, nextInDir, newG, h, current);
                    openSet.Enqueue(nextNode, nextNode.FCost);
                }
            }

            if (bestGoalNode == null)
            {
                return (null, null);
            }

            // 回溯重构路径与时间预约序列
            List<Point> path = new List<Point>();
            List<TimedPathPoint> timedPath = new List<TimedPathPoint>();
            STNode curr = bestGoalNode;

            bool isGoal = true;
            while (curr != null)
            {
                Point pt = new Point(curr.X, curr.Y);
                path.Add(pt);
                // 终点网格持续保守锁定 (直至未来 1000s)，确保上层 CBS/ECBS 及受控区域准确感知终点持续占位，杜绝后车穿行堵塞
                double exitTime = isGoal
                    ? (curr.PhysTime + 1000.0)
                    : (curr.PhysTime + StepPhysicalDuration + ConservativeBuffer);
                timedPath.Add(new TimedPathPoint(pt, curr.PhysTime, exitTime));
                isGoal = false;
                curr = curr.Parent;
            }

            path.Reverse();
            timedPath.Reverse();
            return (path, timedPath);
        }

        private static double CalculateHeuristic(Point current, Point target, Point currentDir, double suboptimality)
        {
            int dx = Math.Abs(current.X - target.X);
            int dy = Math.Abs(current.Y - target.Y);
            double manhattan = (dx + dy) * 1.0005;

            double turnPred = 0.0;
            if (dx > 0 && dy > 0)
            {
                turnPred += 1.0;
            }

            if (currentDir != Point.Empty)
            {
                bool alignX = (currentDir.X != 0 && (target.X - current.X) * currentDir.X > 0);
                bool alignY = (currentDir.Y != 0 && (target.Y - current.Y) * currentDir.Y > 0);
                if (!alignX && !alignY)
                {
                    turnPred += 1.0;
                }
            }

            return (manhattan + turnPred * TurnPenaltyCost) * suboptimality;
        }

        private sealed class STNode
        {
            public int X;
            public int Y;
            public int Time;         // 离散时间步
            public double PhysTime;  // 连续物理时间累计 (秒)
            public Point InDir;
            public double GCost;
            public double HCost;
            public double FCost => GCost + HCost;
            public STNode Parent;

            public STNode(int x, int y, int time, double physTime, Point inDir, double g, double h, STNode parent = null)
            {
                X = x;
                Y = y;
                Time = time;
                PhysTime = physTime;
                InDir = inDir;
                GCost = g;
                HCost = h;
                Parent = parent;
            }
        }

        private static int GetStateKey(int x, int y, int time)
        {
            return (time << 20) ^ (x << 10) ^ y;
        }
    }

    /// <summary>
    /// 多智能体调度规划器，集成了 CBS、ECBS、CA* 与 WHCA* 四大主流算法。
    /// 统一调用底层带保守时间区间预约的 SpaceTimeAStar 核心。
    /// </summary>
    public static class MultiAgentPathPlanning
    {
        public static MultiAgentPlanResult Plan(
            IList<AgentTask> tasks,
            int gridCols,
            int gridRows,
            MultiAgentAlgorithm algorithm,
            int timeoutMs = 4000,
            int whcaWindowK = 18,
            double ecbsSuboptimality = 1.30)
        {
            Stopwatch sw = Stopwatch.StartNew();
            MultiAgentPlanResult result = new MultiAgentPlanResult();

            if (tasks == null || tasks.Count == 0)
            {
                result.Success = true;
                result.Message = "无任务需求";
                return result;
            }

            try
            {
                switch (algorithm)
                {
                    case MultiAgentAlgorithm.CAStar:
                        result = PlanIntervalReservation(tasks, gridCols, gridRows, sw, timeoutMs, windowSize: 0);
                        break;

                    case MultiAgentAlgorithm.WHCAStar:
                        result = PlanIntervalReservation(tasks, gridCols, gridRows, sw, timeoutMs, windowSize: Math.Max(4, whcaWindowK));
                        break;

                    case MultiAgentAlgorithm.ECBS:
                        result = PlanCBS(tasks, gridCols, gridRows, sw, timeoutMs, isEnhanced: true, suboptimality: Math.Max(1.0, ecbsSuboptimality));
                        break;

                    case MultiAgentAlgorithm.CBS:
                    default:
                        result = PlanCBS(tasks, gridCols, gridRows, sw, timeoutMs, isEnhanced: false, suboptimality: 1.0);
                        break;
                }
            }
            catch (Exception ex)
            {
                result.Success = false;
                result.IsTimeout = false;
                result.Message = "调度异常: " + ex.Message;
            }

            sw.Stop();
            result.ElapsedMilliseconds = sw.ElapsedMilliseconds;
            return result;
        }

        #region CA* 与 WHCA* (基于连续保守时间区间预约)

        private struct EdgeInterval
        {
            public Point From;
            public Point To;
            public double EntryTime;
            public double ExitTime;
            public int AgentId;

            public EdgeInterval(Point from, Point to, double entry, double exit, int agentId)
            {
                From = from;
                To = to;
                EntryTime = entry;
                ExitTime = exit;
                AgentId = agentId;
            }

            public bool Overlaps(double otherEntry, double otherExit, double margin = 0.60)
            {
                // 引入防时间漂移的安全时间裕量：两动作即便在浮点时间上相差微小时差，只要间隔小于 margin，亦严格判定为冲突
                return Math.Max(EntryTime - margin, otherEntry) < Math.Min(ExitTime + margin, otherExit);
            }
        }

        private static MultiAgentPlanResult PlanIntervalReservation(
            IList<AgentTask> tasks,
            int gridCols,
            int gridRows,
            Stopwatch sw,
            int timeoutMs,
            int windowSize)
        {
            MultiAgentPlanResult res = new MultiAgentPlanResult();
            int n = tasks.Count;

            // 全局网格保守占用时间区间集合：GridKey -> List<TimeInterval> (顶点预约表)
            Dictionary<int, List<TimeInterval>> gridIntervals = new Dictionary<int, List<TimeInterval>>();

            // 全局边保守占用时间区间集合：UndirectedEdgeKey -> List<EdgeInterval> (边预约表)
            Dictionary<long, List<EdgeInterval>> edgeIntervals = new Dictionary<long, List<EdgeInterval>>();

            // 1. 全量出生点锁定：将所有机器人初始网格在未来预先登记为永久占用，直到各自出发时释放
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

            // 冲突判定委托：校验在目标网格及跨越边上是否与其它智能体产生时空冲突（包含顶点冲突与边冲突）
            bool IsConflict(int agentId, Point from, Point to, double enterTime, double leaveTime)
            {
                // 1. 专属目标格排他性保护：任何智能体严禁借道其他智能体的专属目标栅格，从拓扑根源杜绝停泊堵塞
                for (int j = 0; j < n; j++)
                {
                    if (j != agentId && tasks[j].Target == to && tasks[agentId].Target != to)
                    {
                        return true; // 目标格为他人专属终点车位，严禁作为途经点借道穿行
                    }
                }

                if (windowSize > 0 && enterTime > windowSize * SpaceTimeAStar.StepPhysicalDuration)
                {
                    return false;
                }

                // 2. 顶点冲突判定：校验目标网格 to 在 [enterTime, leaveTime] 是否已被预约
                int toKey = GetCellKey(to.X, to.Y);
                if (gridIntervals.TryGetValue(toKey, out var list))
                {
                    for (int k = 0; k < list.Count; k++)
                    {
                        TimeInterval iv = list[k];
                        if (iv.AgentId != agentId && iv.Overlaps(enterTime, leaveTime))
                        {
                            return true;
                        }
                    }
                }

                // 3. 边冲突判定：若为移动动作 (from != to)，校验对穿边冲突 (Swap Conflict)
                if (from != to)
                {
                    long edgeKey = GetUndirectedEdgeKey(from, to);
                    if (edgeIntervals.TryGetValue(edgeKey, out var eList))
                    {
                        for (int k = 0; k < eList.Count; k++)
                        {
                            EdgeInterval eIv = eList[k];
                            // 校验是否有其他智能体在相向方向 (to -> from)
                            // 单向走廊互斥锁定：引入 2.50s 宽域防时间漂移安全缓冲，绝对杜绝微小时差对向穿行
                            if (eIv.AgentId != agentId && eIv.From == to && eIv.To == from && eIv.Overlaps(enterTime, leaveTime, 2.50))
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
                if (sw.ElapsedMilliseconds > timeoutMs)
                {
                    res.Success = false;
                    res.IsTimeout = true;
                    res.Message = "多机调度超时：检测到死锁或阻塞不可解！";
                    return res;
                }

                AgentTask task = tasks[i];
                int startKey = GetCellKey(task.Start.X, task.Start.Y);

                // 移除智能体 i 自身出生点的未来占位，允许其自身出发
                if (gridIntervals.TryGetValue(startKey, out var startList))
                {
                    startList.RemoveAll(iv => iv.AgentId == i);
                }

                var (path, timedPath) = SpaceTimeAStar.SearchPath(
                    task.Start,
                    task.Target,
                    gridCols,
                    gridRows,
                    task.AgentId,
                    IsConflict,
                    SpaceTimeAStar.MaxTimeSteps);

                if (path == null || path.Count == 0)
                {
                    res.Success = false;
                    res.IsTimeout = true;
                    res.Message = $"智能体 {i + 1} 规划死锁：无可行无冲突路径，调度失败！";
                    return res;
                }

                // 将该智能体的真实时间预约区间写入全局预订表 (包含顶点预约与边预约)
                for (int p = 0; p < timedPath.Count; p++)
                {
                    TimedPathPoint tp = timedPath[p];
                    int cKey = GetCellKey(tp.Grid.X, tp.Grid.Y);
                    if (!gridIntervals.TryGetValue(cKey, out var list))
                    {
                        list = new List<TimeInterval>();
                        gridIntervals[cKey] = list;
                    }

                    // 终点持续保守锁定
                    double exitTime = (p == timedPath.Count - 1) ? 1000.0 : tp.PlannedExitTime;
                    list.Add(new TimeInterval(tp.PlannedEntryTime, exitTime, i));
                }

                for (int p = 0; p < timedPath.Count - 1; p++)
                {
                    Point u = timedPath[p].Grid;
                    Point v = timedPath[p + 1].Grid;
                    if (u != v)
                    {
                        long edgeKey = GetUndirectedEdgeKey(u, v);
                        if (!edgeIntervals.TryGetValue(edgeKey, out var eList))
                        {
                            eList = new List<EdgeInterval>();
                            edgeIntervals[edgeKey] = eList;
                        }

                        double edgeEntry = timedPath[p].PlannedEntryTime;
                        double edgeExit = Math.Min(timedPath[p + 1].PlannedExitTime, timedPath[p + 1].PlannedEntryTime + SpaceTimeAStar.StepPhysicalDuration + SpaceTimeAStar.ConservativeBuffer);
                        eList.Add(new EdgeInterval(u, v, edgeEntry, edgeExit, i));
                    }
                }

                res.AgentPaths.Add(path);
                res.TimedSchedules.Add(timedPath);
            }

            res.Success = true;
            string algoName = (windowSize > 0) ? "WHCA*" : "CA*";
            res.Message = $"{algoName} 成功规划 {n} 条带保守时间区间的预约路径";
            return res;
        }

        #endregion

        #region CBS 与 ECBS 算法

        private struct Conflict
        {
            public int AgentA;
            public int AgentB;
            public bool IsEdgeConflict;
            public Point Loc;
            public Point LocTo;
            public double OverlapStart;
            public double OverlapEnd;
            public double AgentAStart;
            public double AgentAEnd;
            public double AgentBStart;
            public double AgentBEnd;
        }

        private struct CBSConstraint
        {
            public int AgentId;
            public bool IsEdge;
            public Point Loc;
            public Point LocTo;
            public double StartTime;
            public double EndTime;
        }

        private sealed class CTNode
        {
            public List<List<Point>> Paths;
            public List<List<TimedPathPoint>> TimedPaths;
            public List<CBSConstraint> Constraints;
            public int TotalCost;
            public int ConflictCount;

            public CTNode(int n)
            {
                Paths = new List<List<Point>>(n);
                TimedPaths = new List<List<TimedPathPoint>>(n);
                Constraints = new List<CBSConstraint>();
            }

            public CTNode Clone()
            {
                CTNode copy = new CTNode(Paths.Count);
                for (int i = 0; i < Paths.Count; i++)
                {
                    copy.Paths.Add(new List<Point>(Paths[i]));
                    copy.TimedPaths.Add(new List<TimedPathPoint>(TimedPaths[i]));
                }
                copy.Constraints = new List<CBSConstraint>(Constraints);
                copy.TotalCost = TotalCost;
                copy.ConflictCount = ConflictCount;
                return copy;
            }
        }

        private static MultiAgentPlanResult PlanCBS(
            IList<AgentTask> tasks,
            int gridCols,
            int gridRows,
            Stopwatch sw,
            int timeoutMs,
            bool isEnhanced,
            double suboptimality = 1.0)
        {
            MultiAgentPlanResult res = new MultiAgentPlanResult();
            int n = tasks.Count;

            CTNode root = new CTNode(n);
            int initialCost = 0;

            // 专属目标格排他性保护委托：根节点初始规划严禁借道其他智能体的目标车位
            bool IsRootTargetExclusive(int aid, Point from, Point to, double enterTime, double leaveTime)
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
                if (sw.ElapsedMilliseconds > timeoutMs)
                {
                    res.Success = false;
                    res.IsTimeout = true;
                    res.Message = "CBS 初始规划超时 (死锁)！";
                    return res;
                }

                var (p, tp) = SpaceTimeAStar.SearchPath(
                    tasks[i].Start,
                    tasks[i].Target,
                    gridCols,
                    gridRows,
                    i,
                    IsRootTargetExclusive,
                    SpaceTimeAStar.MaxTimeSteps,
                    suboptimality);

                if (p == null || p.Count == 0)
                {
                    res.Success = false;
                    res.IsTimeout = true;
                    res.Message = $"智能体 {i + 1} 独立无解！";
                    return res;
                }

                root.Paths.Add(p);
                root.TimedPaths.Add(tp);
                initialCost += p.Count;
            }

            root.TotalCost = initialCost;
            root.ConflictCount = CountIntervalConflicts(root.TimedPaths);

            PriorityQueue<CTNode, double> openTree = new PriorityQueue<CTNode, double>();
            openTree.Enqueue(root, isEnhanced ? (root.TotalCost + root.ConflictCount * 5.0) : root.TotalCost);

            int expandedNodes = 0;
            const int maxExpandedNodes = 400;

            while (openTree.Count > 0)
            {
                expandedNodes++;

                if (sw.ElapsedMilliseconds > timeoutMs || expandedNodes > maxExpandedNodes)
                {
                    if (isEnhanced || tasks.Count >= 15)
                    {
                        return PlanIntervalReservation(tasks, gridCols, gridRows, sw, timeoutMs, windowSize: 0);
                    }

                    res.Success = false;
                    res.IsTimeout = true;
                    res.Message = "CBS 调度超时：冲突树组合爆炸，任务失败！";
                    return res;
                }

                CTNode node = openTree.Dequeue();

                if (!FindNextIntervalConflict(node.TimedPaths, out Conflict conflict))
                {
                    res.Success = true;
                    res.AgentPaths = node.Paths;
                    res.TimedSchedules = node.TimedPaths;
                    string name = isEnhanced ? "ECBS" : "CBS";
                    res.Message = $"{name} 成功规划 {n} 条无冲突时空区间路径 (展开{expandedNodes}节点)";
                    return res;
                }

                for (int b = 0; b < 2; b++)
                {
                    int agentIndex = (b == 0) ? conflict.AgentA : conflict.AgentB;
                    CTNode child = node.Clone();

                    // 核心关键：对于边冲突，约束区间必须是对方穿越该边的全时段，彻底杜绝轻微错位钻空子迎头对穿！
                    double cStart = conflict.IsEdgeConflict
                        ? (b == 0 ? conflict.AgentBStart : conflict.AgentAStart)
                        : conflict.OverlapStart;
                    double cEnd = conflict.IsEdgeConflict
                        ? (b == 0 ? conflict.AgentBEnd : conflict.AgentAEnd)
                        : conflict.OverlapEnd;

                    CBSConstraint newC = new CBSConstraint
                    {
                        AgentId = agentIndex,
                        IsEdge = conflict.IsEdgeConflict,
                        Loc = conflict.IsEdgeConflict ? (b == 0 ? conflict.Loc : conflict.LocTo) : conflict.Loc,
                        LocTo = conflict.IsEdgeConflict ? (b == 0 ? conflict.LocTo : conflict.Loc) : Point.Empty,
                        StartTime = cStart,
                        EndTime = cEnd
                    };
                    child.Constraints.Add(newC);

                    bool IsViolated(int aid, Point from, Point to, double enterTime, double leaveTime)
                    {
                        if (aid != agentIndex) return false;

                        // 专属目标格排他性保护：重新规划时同样严禁借道其他智能体的目标车位
                        for (int j = 0; j < n; j++)
                        {
                            if (j != aid && tasks[j].Target == to && tasks[aid].Target != to)
                            {
                                return true;
                            }
                        }

                        for (int k = 0; k < child.Constraints.Count; k++)
                        {
                            CBSConstraint c = child.Constraints[k];
                            if (c.AgentId != agentIndex) continue;

                            if (c.IsEdge)
                            {
                                // 边约束：禁止在该时间段内从 c.Loc 移动到 c.LocTo
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
                                // 顶点约束：禁止在该时间段内占用目标点 c.Loc
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
                        tasks[agentIndex].Start,
                        tasks[agentIndex].Target,
                        gridCols,
                        gridRows,
                        agentIndex,
                        IsViolated,
                        SpaceTimeAStar.MaxTimeSteps,
                        suboptimality);

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

            res.Success = false;
            res.IsTimeout = true;
            res.Message = "多机调度无可行解 (死锁)！";
            return res;
        }

        private static bool FindNextIntervalConflict(List<List<TimedPathPoint>> schedules, out Conflict conflict)
        {
            conflict = default;
            bool found = false;
            double minOverlapStart = double.MaxValue;

            for (int i = 0; i < schedules.Count; i++)
            {
                var schedI = schedules[i];
                for (int j = i + 1; j < schedules.Count; j++)
                {
                    var schedJ = schedules[j];

                    // 1. 顶点冲突检测 (同一时间区间占用同一网格顶点)
                    for (int pi = 0; pi < schedI.Count; pi++)
                    {
                        TimedPathPoint ptI = schedI[pi];
                        for (int pj = 0; pj < schedJ.Count; pj++)
                        {
                            TimedPathPoint ptJ = schedJ[pj];
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
                                            LocTo = Point.Empty,
                                            OverlapStart = overlapStart,
                                            OverlapEnd = overlapEnd
                                        };
                                        found = true;
                                    }
                                }
                            }
                        }
                    }

                    // 2. 边冲突检测 (同一时间区间沿同一无向边相向对穿 / 交换位置 Swap Conflict)
                    for (int pi = 0; pi < schedI.Count - 1; pi++)
                    {
                        Point uI = schedI[pi].Grid;
                        Point vI = schedI[pi + 1].Grid;
                        if (uI == vI) continue;

                        for (int pj = 0; pj < schedJ.Count - 1; pj++)
                        {
                            Point uJ = schedJ[pj].Grid;
                            Point vJ = schedJ[pj + 1].Grid;
                            if (uJ == vJ) continue;

                            // 互为反向边：i 走 uI->vI，j 走 vI->uI
                            if (uI == vJ && vI == uJ)
                            {
                                double startI = schedI[pi].PlannedEntryTime;
                                double exitI = Math.Min(schedI[pi + 1].PlannedExitTime, schedI[pi + 1].PlannedEntryTime + SpaceTimeAStar.StepPhysicalDuration + SpaceTimeAStar.ConservativeBuffer);
                                double startJ = schedJ[pj].PlannedEntryTime;
                                double exitJ = Math.Min(schedJ[pj + 1].PlannedExitTime, schedJ[pj + 1].PlannedEntryTime + SpaceTimeAStar.StepPhysicalDuration + SpaceTimeAStar.ConservativeBuffer);

                                // 单向走廊互斥锁定：引入 2.0s 宽域防时间漂移裕量及离散步长同步保底，彻底杜绝对向穿行
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
                                            AgentAStart = startI - edgeDriftMargin,
                                            AgentAEnd = exitI + edgeDriftMargin,
                                            AgentBStart = startJ - edgeDriftMargin,
                                            AgentBEnd = exitJ + edgeDriftMargin
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

        private static int CountIntervalConflicts(List<List<TimedPathPoint>> schedules)
        {
            int count = 0;
            for (int i = 0; i < schedules.Count; i++)
            {
                var schedI = schedules[i];
                for (int j = i + 1; j < schedules.Count; j++)
                {
                    var schedJ = schedules[j];

                    // 1. 统计顶点冲突
                    for (int pi = 0; pi < schedI.Count; pi++)
                    {
                        for (int pj = 0; pj < schedJ.Count; pj++)
                        {
                            if (schedI[pi].Grid == schedJ[pj].Grid)
                            {
                                if (Math.Max(schedI[pi].PlannedEntryTime, schedJ[pj].PlannedEntryTime) <
                                    Math.Min(schedI[pi].PlannedExitTime, schedJ[pj].PlannedExitTime))
                                {
                                    count++;
                                }
                            }
                        }
                    }

                    // 2. 统计边冲突 (相向对穿，含单向走廊互斥防漂移时间窗)
                    for (int pi = 0; pi < schedI.Count - 1; pi++)
                    {
                        Point uI = schedI[pi].Grid;
                        Point vI = schedI[pi + 1].Grid;
                        if (uI == vI) continue;

                        for (int pj = 0; pj < schedJ.Count - 1; pj++)
                        {
                            Point uJ = schedJ[pj].Grid;
                            Point vJ = schedJ[pj + 1].Grid;
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
                                    count++;
                                }
                            }
                        }
                    }
                }
            }
            return count;
        }

        #endregion

        #region 辅助哈希工具

        private static int GetCellKey(int x, int y)
        {
            return (x << 16) | (y & 0xFFFF);
        }

        private static long GetUndirectedEdgeKey(Point u, Point v)
        {
            int k1 = GetCellKey(u.X, u.Y);
            int k2 = GetCellKey(v.X, v.Y);
            if (k1 > k2)
            {
                int t = k1;
                k1 = k2;
                k2 = t;
            }
            return ((long)k1 << 32) | (uint)k2;
        }

        #endregion
    }
}
