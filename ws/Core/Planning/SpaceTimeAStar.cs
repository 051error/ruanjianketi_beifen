using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;

namespace WarehouseGridSimulation.Core.Planning
{
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
        public const double DefaultCellSize = 0.55;
        public const double StepPhysicalDuration = DefaultCellSize / DefaultSpeed; // 约 0.367 秒/格
        public const double TurnPhysicalDuration = 0.50;                          // 原地旋转约 0.50 秒
        public const double ConservativeBuffer = 0.25;                           // 保守安全时间裕量 0.25 秒

        private static readonly (int X, int Y)[] DirectionsWithWait = new (int, int)[]
        {
            (0, 0),   // 原地等待
            (0, -1),  // 上
            (1, 0),   // 右
            (0, 1),   // 下
            (-1, 0)   // 左
        };

        public static (List<GridPoint> Path, List<TimedPathPoint> TimedPath) SearchPath(
            GridMap map,
            GridPoint start,
            GridPoint target,
            int agentId,
            Func<int, GridPoint, GridPoint, double, double, bool> isIntervalConflict,
            int maxSteps = MaxTimeSteps,
            double suboptimality = 1.0,
            Func<GridPoint, bool> isDynamicBlocked = null)
        {
            // 执行带保守连续时间区间检验与动态障碍过滤的时空 A* 搜索。
            // map: 仓储栅格地图; start: 规划起点网格; target: 规划终点网格; agentId: 智能体全局编号; isIntervalConflict: 时空碰撞冲突校验委托; maxSteps: 最大时间步长上限; suboptimality: 次优启发式松弛因子; isDynamicBlocked: 动态障碍物过滤委托。
            if (start == target)
            {
                var p = new List<GridPoint> { start };
                var tp = new List<TimedPathPoint> { new TimedPathPoint(start, 0.0, 1000.0) };
                return (p, tp);
            }

            int gridCols = map?.Width ?? 30;
            int gridRows = map?.Height ?? 30;

            var openSet = new PriorityQueue<STNode, double>();
            var bestCostAtState = new Dictionary<int, double>();

            double initialH = CalculateHeuristic(start, target, (0, 0), suboptimality);
            var startNode = new STNode(start.X, start.Y, 0, 0.0, (0, 0), 0.0, initialH);
            openSet.Enqueue(startNode, startNode.FCost);
            bestCostAtState[GetStateKey(start.X, start.Y, 0)] = 0.0;

            STNode bestGoalNode = null;

            while (openSet.Count > 0)
            {
                var current = openSet.Dequeue();

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
                var currPt = new GridPoint(current.X, current.Y);

                for (int d = 0; d < DirectionsWithWait.Length; d++)
                {
                    int nx = current.X + DirectionsWithWait[d].X;
                    int ny = current.Y + DirectionsWithWait[d].Y;

                    if (nx < 0 || nx >= gridCols || ny < 0 || ny >= gridRows)
                    {
                        continue;
                    }

                    var nextPt = new GridPoint(nx, ny);
                    if (map != null && map.IsObstacle(nextPt))
                    {
                        continue;
                    }

                    if (isDynamicBlocked != null && isDynamicBlocked(nextPt))
                    {
                        continue;
                    }

                    // 严禁 180° 倒车掉头摆动：动作不得退回刚离开的父节点网格，避让必须通过原地等待或侧向绕行实现
                    if (current.Parent != null && d != 0 && nx == current.Parent.X && ny == current.Parent.Y)
                    {
                        continue;
                    }

                    (int X, int Y) outDir = DirectionsWithWait[d];

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
                        if (current.InDir != (0, 0) && current.InDir != outDir)
                        {
                            actionDuration += TurnPhysicalDuration;
                            stepCost += TurnPenaltyCost;
                        }
                    }

                    double enterTime = current.PhysTime;
                    double leaveTime = enterTime + actionDuration + ConservativeBuffer;

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
                    var nextInDir = (d == 0) ? current.InDir : outDir;
                    double nextPhysTime = current.PhysTime + actionDuration;
                    double h = CalculateHeuristic(nextPt, target, nextInDir, suboptimality);

                    var nextNode = new STNode(nx, ny, nextStep, nextPhysTime, nextInDir, newG, h, current);
                    openSet.Enqueue(nextNode, nextNode.FCost);
                }
            }

            if (bestGoalNode == null)
            {
                return (null, null);
            }

            // 回溯重构路径与时间预约序列
            var path = new List<GridPoint>();
            var timedPath = new List<TimedPathPoint>();
            var currNode = bestGoalNode;

            bool isGoal = true;
            while (currNode != null)
            {
                var pt = new GridPoint(currNode.X, currNode.Y);
                path.Add(pt);
                double exitTime = isGoal
                    ? (currNode.PhysTime + 1000.0)
                    : (currNode.PhysTime + StepPhysicalDuration + ConservativeBuffer);
                timedPath.Add(new TimedPathPoint(pt, currNode.PhysTime, exitTime));
                isGoal = false;
                currNode = currNode.Parent;
            }

            path.Reverse();
            timedPath.Reverse();
            return (path, timedPath);
        }

        private static double CalculateHeuristic(GridPoint current, GridPoint target, (int X, int Y) currentDir, double suboptimality)
        {
            // 计算带有航向转向惩罚预估的时空启发式曼哈顿距离。
            // current: 当前网格坐标; currentDir: 驶入方向向量。
            int dx = Math.Abs(current.X - target.X);
            int dy = Math.Abs(current.Y - target.Y);
            double manhattan = (dx + dy) * 1.0005;

            double turnPred = 0.0;
            if (dx > 0 && dy > 0)
            {
                turnPred += 1.0;
            }

            if (currentDir != (0, 0))
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

        private static int GetStateKey(int x, int y, int time)
        {
            // 将网格坐标与离散时间步编码为紧凑唯一的哈希键。
            return (time << 16) | ((x & 0xFF) << 8) | (y & 0xFF);
        }

        private sealed class STNode
        {
            public int X;
            public int Y;
            public int Time;
            public double PhysTime;
            public (int X, int Y) InDir;
            public double GCost;
            public double HCost;
            public double FCost => GCost + HCost;
            public STNode Parent;

            public STNode(int x, int y, int time, double physTime, (int X, int Y) inDir, double g, double h, STNode parent = null)
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
    }
}
