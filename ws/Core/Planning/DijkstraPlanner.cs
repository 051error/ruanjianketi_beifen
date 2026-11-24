using System;
using WarehouseGridSimulation.Core.Geometry;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 标准四方向栅格 Dijkstra 路径规划器。
    /// 继承自 AStarPlanner 并复用全部图搜索、集合管理与路径回溯逻辑，零冗余代码。
    /// 启发式代价值恒为 0，按实际走过代价进行一致代价广度搜索。
    /// </summary>
    public class DijkstraPlanner : AStarPlanner
    {
        protected override float ComputeHeuristic(GridPoint current, GridPoint? moveDir, GridPoint goal)
        {
            // 计算 Dijkstra 启发式代价值 (恒为零，纯实际移动代价优先)。
            return 0f;
        }
    }
}
