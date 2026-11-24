using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 路径规划输出结果。
    /// </summary>
    public class PathPlanResult
    {
        public bool Success { get; }
        public IReadOnlyList<GridPoint> Path { get; }
        public string ErrorMessage { get; }
        public float Cost { get; }

        public PathPlanResult(bool success, IReadOnlyList<GridPoint> path, string errorMessage = null, float cost = 0f)
        {
            Success = success;
            Path = path ?? Array.Empty<GridPoint>();
            ErrorMessage = errorMessage ?? string.Empty;
            Cost = cost;
        }

        public static PathPlanResult Fail(string reason) => new PathPlanResult(false, null, reason);
        public static PathPlanResult Ok(IReadOnlyList<GridPoint> path, float cost) => new PathPlanResult(true, path, null, cost);
    }

    /// <summary>
    /// 路径规划器可插拔接口。
    /// </summary>
    public interface IPathPlanner
    {
        PathPlanResult PlanPath(
            GridMap map,
            GridPoint start,
            GridPoint goal,
            Func<GridPoint, bool> isDynamicBlocked = null);
    }
}
