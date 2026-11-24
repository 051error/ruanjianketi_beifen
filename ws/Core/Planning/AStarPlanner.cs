using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 标准四方向栅格 A* 路径规划器。
    /// 包含严格的结果校验：
    /// - 起终点合法性校验；
    /// - 连续性校验 (相连节点必为曼哈顿邻居)；
    /// - 无障碍穿透校验。
    /// </summary>
    public class AStarPlanner : IPathPlanner
    {
        private static readonly GridPoint[] Directions = new[]
        {
            new GridPoint(0, -1), // 上
            new GridPoint(1, 0),  // 右
            new GridPoint(0, 1),  // 下
            new GridPoint(-1, 0)  // 左
        };

        public PathPlanResult PlanPath(
            GridMap map,
            GridPoint start,
            GridPoint goal,
            Func<GridPoint, bool> isDynamicBlocked = null)
        {
            if (map == null) throw new ArgumentNullException(nameof(map));

            // 1. 起终点校验
            if (!map.IsInside(start))
                return PathPlanResult.Fail($"起点 {start} 超出地图边界");
            if (!map.IsInside(goal))
                return PathPlanResult.Fail($"终点 {goal} 超出地图边界");
            if (map.IsObstacle(start))
                return PathPlanResult.Fail($"起点 {start} 位于障碍物内");
            if (map.IsObstacle(goal))
                return PathPlanResult.Fail($"终点 {goal} 位于障碍物内");

            if (isDynamicBlocked != null && isDynamicBlocked(goal))
                return PathPlanResult.Fail($"终点 {goal} 当前被动态占用不可通行");

            if (start == goal)
            {
                return PathPlanResult.Ok(new List<GridPoint> { start }, 0f);
            }

            var openSet = new PriorityQueue<GridPoint, float>();
            var cameFrom = new Dictionary<GridPoint, GridPoint>();
            var costSoFar = new Dictionary<GridPoint, float>();
            var closedSet = new HashSet<GridPoint>();

            costSoFar[start] = 0f;
            float initialH = ComputeHeuristic(start, null, goal);
            openSet.Enqueue(start, initialH);

            while (openSet.Count > 0)
            {
                GridPoint current = openSet.Dequeue();

                if (current == goal)
                {
                    var reconstructed = ReconstructPath(cameFrom, current);
                    // 校验规划结果
                    if (ValidatePath(map, reconstructed, out string validationError))
                    {
                        return PathPlanResult.Ok(reconstructed, costSoFar[current]);
                    }
                    return PathPlanResult.Fail($"路径校验不通过: {validationError}");
                }

                if (!closedSet.Add(current))
                    continue;

                float currentG = costSoFar[current];

                foreach (var dir in Directions)
                {
                    GridPoint neighbor = current + dir;

                    if (!map.IsInside(neighbor) || map.IsObstacle(neighbor))
                        continue;

                    // 若设置了动态阻挡检测，且不是目标点，检查是否被阻挡
                    if (isDynamicBlocked != null && neighbor != goal && isDynamicBlocked(neighbor))
                        continue;

                    if (closedSet.Contains(neighbor))
                        continue;

                    float stepCost = 1.0f;
                    float newG = currentG + stepCost;

                    if (!costSoFar.TryGetValue(neighbor, out float bestG) || newG < bestG)
                    {
                        costSoFar[neighbor] = newG;
                        cameFrom[neighbor] = current;
                        float h = ComputeHeuristic(neighbor, dir, goal);
                        openSet.Enqueue(neighbor, newG + h);
                    }
                }
            }

            return PathPlanResult.Fail($"未找到从 {start} 到 {goal} 的可行路径 (目标不可达)");
        }

        private static List<GridPoint> ReconstructPath(Dictionary<GridPoint, GridPoint> cameFrom, GridPoint current)
        {
            var path = new List<GridPoint> { current };
            while (cameFrom.TryGetValue(current, out GridPoint parent))
            {
                path.Add(parent);
                current = parent;
            }
            path.Reverse();
            return path;
        }

        protected virtual float ComputeHeuristic(GridPoint current, GridPoint? moveDir, GridPoint goal)
        {
            // 计算 A* 启发式代价值 (0.3 曼哈顿距离 + 0.7 转向成本)。
            // current: 当前评估节点; moveDir: 驶入当前节点的方向微元矢量; goal: 目标终点。
            float manhattan = current.ManhattanDistance(goal);
            float turnCost = EstimateTurnCost(current, moveDir, goal);
            return 0.3f * manhattan + 0.7f * turnCost;
        }

        private static float EstimateTurnCost(GridPoint current, GridPoint? moveDir, GridPoint goal)
        {
            // 估算从当前节点与朝向到达目标所需的最少转向次数。
            if (current == goal) return 0f;
            int dx = goal.X - current.X;
            int dy = goal.Y - current.Y;

            if (!moveDir.HasValue)
            {
                return (dx == 0 || dy == 0) ? 0f : 1f;
            }

            var dir = moveDir.Value;
            if (dy == 0)
            {
                if (dir.Y != 0) return 1f;
                return Math.Sign(dir.X) == Math.Sign(dx) ? 0f : 2f;
            }
            if (dx == 0)
            {
                if (dir.X != 0) return 1f;
                return Math.Sign(dir.Y) == Math.Sign(dy) ? 0f : 2f;
            }

            bool alignsX = dir.X != 0 && Math.Sign(dir.X) == Math.Sign(dx);
            bool alignsY = dir.Y != 0 && Math.Sign(dir.Y) == Math.Sign(dy);
            return (alignsX || alignsY) ? 1f : 2f;
        }

        private static bool ValidatePath(GridMap map, IReadOnlyList<GridPoint> path, out string error)
        {
            error = null;
            if (path == null || path.Count == 0)
            {
                error = "路径为空";
                return false;
            }

            for (int i = 0; i < path.Count; i++)
            {
                if (!map.IsInside(path[i]))
                {
                    error = $"节点 #{i} ({path[i]}) 超出地图边界";
                    return false;
                }
                if (map.IsObstacle(path[i]))
                {
                    error = $"节点 #{i} ({path[i]}) 穿越障碍物";
                    return false;
                }
                if (i > 0)
                {
                    int dist = path[i].ManhattanDistance(path[i - 1]);
                    if (dist != 1)
                    {
                        error = $"节点 #{i - 1} 与节点 #{i} 不连续 (曼哈顿跨度 {dist} != 1)";
                        return false;
                    }
                }
            }
            return true;
        }
    }
}
