using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;
using WarehouseGridSimulation.Core.Robots;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 对角专属目标网格分配生成器。
    /// 负责为多机调度与无尽模式计算生成位于机器人对角半区未被占用的新目标网格。
    /// </summary>
    public static class DiagonalTargetAllocator
    {
        public static GridPoint FindDiagonalTarget(
            GridMap map,
            SimulatedRobot robot,
            ISet<GridPoint> usedTargets,
            Random random = null)
        {
            // 为机器人寻找并分配对其所属象限对角半区未被占用的可行目标网格。
            // map: 仓储栅格地图; robot: 目标机器人实体; usedTargets: 当前已被其他智能体占用的目标点集合; random: 可选的伪随机数生成器。
            if (map == null || robot == null)
            {
                return robot?.ConfirmedGrid ?? new GridPoint(0, 0);
            }

            var rnd = random ?? Random.Shared;
            int midX = map.Width / 2;
            int midY = map.Height / 2;

            int robotQuadX = (robot.ConfirmedGrid.X < midX) ? 0 : 1;
            int robotQuadY = (robot.ConfirmedGrid.Y < midY) ? 0 : 1;
            int diagQuadX = 1 - robotQuadX;
            int diagQuadY = 1 - robotQuadY;

            int minX = (diagQuadX == 0) ? 0 : midX;
            int maxX = (diagQuadX == 0) ? (midX - 1) : map.Width - 1;
            int minY = (diagQuadY == 0) ? 0 : midY;
            int maxY = (diagQuadY == 0) ? (midY - 1) : map.Height - 1;

            var candidates = new List<GridPoint>();
            var fallbacks = new List<GridPoint>();

            for (int x = minX; x <= maxX; x++)
            {
                for (int y = minY; y <= maxY; y++)
                {
                    var pt = new GridPoint(x, y);
                    if (usedTargets != null && usedTargets.Contains(pt)) continue;
                    if (map.IsObstacle(pt)) continue;

                    fallbacks.Add(pt);
                    if (Math.Abs(x - robot.ConfirmedGrid.X) > 10 || Math.Abs(y - robot.ConfirmedGrid.Y) > 10)
                    {
                        candidates.Add(pt);
                    }
                }
            }

            if (candidates.Count > 0)
            {
                return candidates[rnd.Next(candidates.Count)];
            }
            if (fallbacks.Count > 0)
            {
                return fallbacks[rnd.Next(fallbacks.Count)];
            }
            return robot.ConfirmedGrid;
        }
    }
}
