using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Geometry;

namespace WarehouseGridSimulation.Core.Planning
{
    /// <summary>
    /// 高层导航请求执行结果。
    /// </summary>
    public class NavigationResult
    {
        public bool Success { get; }
        public string Message { get; }
        public IReadOnlyList<GridPoint> Path { get; }

        public NavigationResult(bool success, string message, IReadOnlyList<GridPoint> path = null)
        {
            // 初始化导航结果实例。
            // success: 导航启动是否成功; message: 结果状态或失败提示信息; path: 成功规划生成的离散路径节点列表。
            Success = success;
            Message = message ?? string.Empty;
            Path = path ?? Array.Empty<GridPoint>();
        }

        public static NavigationResult Ok(IReadOnlyList<GridPoint> path, string message = "导航路径下发成功")
        {
            // 创建表示成功的导航结果实例。
            return new NavigationResult(true, message, path);
        }

        public static NavigationResult Fail(string message)
        {
            // 创建表示失败的导航结果实例。
            return new NavigationResult(false, message, null);
        }
    }
}
