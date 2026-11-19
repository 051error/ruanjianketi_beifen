using System;
using System.Collections.Generic;
using System.Drawing;

namespace WarehouseGridSimulation.Skia.Algorithms
{
    /// <summary>
    /// 寻路算法类型。
    /// </summary>
    public enum PathAlgorithm
    {
        /// <summary>
        /// Dijkstra 最短路径算法（无启发式，h = 0）。
        /// </summary>
        Dijkstra = 0,

        /// <summary>
        /// A* 启发式搜索算法（启发式代价计入目标曼哈顿距离与未来转向成本）。
        /// </summary>
        AStar = 1
    }

    /// <summary>
    /// 统一网格寻路规划器，通过同一套搜索逻辑支持 Dijkstra 与考虑转向成本的 A* 算法。
    /// </summary>
    public static class PathPlanning
    {
        /// <summary>
        /// 启发式函数中曼哈顿距离的权重系数（曼哈顿距离与转向成本权重比例为 3:7）。
        /// </summary>
        public const double DistanceWeight = 3.0;

        /// <summary>
        /// 启发式函数中单次转向成本的权重系数（曼哈顿距离与转向成本权重比例为 3:7）。
        /// </summary>
        public const double TurnCostWeight = 7.0;

        private static readonly Point[] Directions = new Point[]
        {
            new Point(0, -1), // 上
            new Point(1, 0),  // 右
            new Point(0, 1),  // 下
            new Point(-1, 0)  // 左
        };

        public static List<Point> FindPath(
            Point start,
            Point target,
            int gridColumns,
            int gridRows,
            PathAlgorithm algorithm,
            Func<Point, bool> isWalkable = null)
        {
            // 使用统一搜索流程规划从起点到目标点的最优网格路径。
            // start: 寻路的起始离散网格坐标。
            // target: 寻路的目标离散网格坐标。
            // gridColumns: 地图网格总列数。
            // gridRows: 地图网格总行数。
            // algorithm: 寻路算法类型（Dijkstra 或 AStar）。
            // isWalkable: 判定指定网格是否可通行的外部条件委托。
            if (start == target)
            {
                return new List<Point> { start };
            }

            if (!IsInsideBounds(start, gridColumns, gridRows) || !IsInsideBounds(target, gridColumns, gridRows))
            {
                return new List<Point>();
            }

            if (isWalkable != null && (!isWalkable(start) || !isWalkable(target)))
            {
                return new List<Point>();
            }

            PriorityQueue<Point, double> openSet = new PriorityQueue<Point, double>();
            Dictionary<Point, Point> cameFrom = new Dictionary<Point, Point>();
            Dictionary<Point, double> costSoFar = new Dictionary<Point, double>();
            HashSet<Point> closedSet = new HashSet<Point>();

            costSoFar[start] = 0.0;
            double initialPriority = CalculateHeuristic(start, target, algorithm, Point.Empty, Point.Empty);
            openSet.Enqueue(start, initialPriority);

            while (openSet.Count > 0)
            {
                Point current = openSet.Dequeue();

                if (current == target)
                {
                    return ReconstructPath(cameFrom, current);
                }

                if (!closedSet.Add(current))
                {
                    continue;
                }

                double currentCost = costSoFar[current];
                bool hasParent = cameFrom.TryGetValue(current, out Point parent);
                Point inDir = hasParent ? new Point(current.X - parent.X, current.Y - parent.Y) : Point.Empty;

                foreach (Point next in GetNeighbors(current, gridColumns, gridRows, isWalkable))
                {
                    if (closedSet.Contains(next))
                    {
                        continue;
                    }

                    Point outDir = new Point(next.X - current.X, next.Y - current.Y);
                    double stepCost = 1.0;

                    // 仅在 A* 算法中将转向物理开销纳入代价计算体系（比例为 1:10）
                    if (algorithm == PathAlgorithm.AStar && hasParent && inDir != outDir)
                    {
                        stepCost += TurnCostWeight;
                    }

                    double newCost = currentCost + stepCost;
                    if (!costSoFar.TryGetValue(next, out double bestCost) || newCost < bestCost)
                    {
                        costSoFar[next] = newCost;
                        cameFrom[next] = current;
                        double priority = newCost + CalculateHeuristic(next, target, algorithm, outDir, inDir);
                        openSet.Enqueue(next, priority);
                    }
                }
            }

            return new List<Point>();
        }

        private static double CalculateHeuristic(
            Point current,
            Point target,
            PathAlgorithm algorithm,
            Point nextDir,
            Point prevDir)
        {
            // 计算当前节点到达目标节点的启发式代价估算值，仅在 A* 算法中按 3:7 比例计入曼哈顿距离与转向成本。
            // current: 当前被评估的网格坐标。
            // nextDir: 当前尝试迈向目标节点的移动方向向量。
            // prevDir: 从父级节点到达当前节点的前进方向向量。
            if (algorithm == PathAlgorithm.Dijkstra)
            {
                return 0.0;
            }

            double deltaX = Math.Abs(current.X - target.X);
            double deltaY = Math.Abs(current.Y - target.Y);
            double manhattanDist = deltaX + deltaY;

            double turnCount = 0.0;

            // 1. 本步转向惩罚：如果从父级方向到这一步的方向发生了改变，产生 1 次转向代价
            if (prevDir != Point.Empty && nextDir != prevDir)
            {
                turnCount += 1.0;
            }

            // 2. 预测未来转向惩罚：若当前点与目标在横纵坐标上均不重合，正交行进未来至少还需要 1 次转向
            if (current.X != target.X && current.Y != target.Y)
            {
                turnCount += 1.0;
            }

            // 曼哈顿距离权重与转向成本权重比例严格为 3:7
            return (DistanceWeight * manhattanDist) + (TurnCostWeight * turnCount);
        }

        private static IEnumerable<Point> GetNeighbors(
            Point node,
            int gridColumns,
            int gridRows,
            Func<Point, bool> isWalkable)
        {
            // 获取指定网格四周上下左右四个方向的合法邻居网格节点。
            // node: 需要探测四周邻居的中心网格坐标。
            for (int i = 0; i < Directions.Length; i++)
            {
                Point next = new Point(node.X + Directions[i].X, node.Y + Directions[i].Y);

                if (IsInsideBounds(next, gridColumns, gridRows))
                {
                    if (isWalkable == null || isWalkable(next))
                    {
                        yield return next;
                    }
                }
            }
        }

        private static List<Point> ReconstructPath(Dictionary<Point, Point> cameFrom, Point current)
        {
            // 通过父节点索引字典自目标点向起点回溯并重构完整路径序列。
            // cameFrom: 记录每个节点前驱父节点的映射字典。
            List<Point> path = new List<Point> { current };
            while (cameFrom.TryGetValue(current, out Point previous))
            {
                path.Add(previous);
                current = previous;
            }

            path.Reverse();
            return path;
        }

        private static bool IsInsideBounds(Point point, int gridColumns, int gridRows)
        {
            // 校验给定的网格坐标是否处于地图边界之内。
            // point: 待校验的网格坐标。
            return point.X >= 0 && point.X < gridColumns && point.Y >= 0 && point.Y < gridRows;
        }
    }
}
