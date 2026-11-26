using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Geometry;

namespace WarehouseGridSimulation.Core.Traffic
{
    /// <summary>
    /// 栅格与无向/有向边预约与交通占用管理器。
    /// 核心安全职责：
    /// 1. 绝对杜绝两车进入同一栅格；
    /// 2. 绝对杜绝两车在同一条边上对向驶入 (Head-on)；
    /// 3. 权威占用必须在车身完全物理退出后方可释放，机器人停止或异常时不自动释放；
    /// 4. 跟踪等待时间与超时事件，避免将死锁或长时间阻塞误判为任务完成。
    /// </summary>
    public class GridReservationManager
    {
        // 栅格绝对占用：Grid -> RobotId
        private readonly Dictionary<GridPoint, int> _occupiedGrids = new Dictionary<GridPoint, int>();
        // 栅格进入预约：Grid -> RobotId
        private readonly Dictionary<GridPoint, int> _reservedGrids = new Dictionary<GridPoint, int>();
        // 边占用预约：UndirectedEdgeKey -> RobotId
        private readonly Dictionary<long, int> _reservedEdges = new Dictionary<long, int>();

        // 机器人等待时间统计 (秒)
        private readonly Dictionary<int, float> _waitTimes = new Dictionary<int, float>();

        public float LongWaitThresholdSeconds { get; set; } = 3.0f;

        public event Action<int, GridPoint, float, string> LongWaitDetected;

        public IReadOnlyDictionary<GridPoint, int> OccupiedGrids => _occupiedGrids;
        public IReadOnlyDictionary<GridPoint, int> ReservedGrids => _reservedGrids;
        public IReadOnlyDictionary<long, int> ReservedEdges => _reservedEdges;

        public void Clear()
        {
            _occupiedGrids.Clear();
            _reservedGrids.Clear();
            _reservedEdges.Clear();
            _waitTimes.Clear();
        }

        public void RegisterRobotInitialGrid(int robotId, GridPoint grid)
        {
            _occupiedGrids[grid] = robotId;
            _waitTimes[robotId] = 0f;
        }

        public void ReleaseRobotReservations(int robotId, GridPoint preserveGrid)
        {
            // 清理指定机器人在旧路径上持有的全部预约栅格与通行边，仅保留当前已占有的物理栅格。
            // robotId: 目标机器人编号; preserveGrid: 机器人当前已确认占有的物理栅格。
            var reservedKeysToRemove = new List<GridPoint>();
            foreach (var kvp in _reservedGrids)
            {
                if (kvp.Value == robotId)
                {
                    reservedKeysToRemove.Add(kvp.Key);
                }
            }
            foreach (var key in reservedKeysToRemove)
            {
                _reservedGrids.Remove(key);
            }

            var edgeKeysToRemove = new List<long>();
            foreach (var kvp in _reservedEdges)
            {
                if (kvp.Value == robotId)
                {
                    edgeKeysToRemove.Add(kvp.Key);
                }
            }
            foreach (var key in edgeKeysToRemove)
            {
                _reservedEdges.Remove(key);
            }

            var occupiedKeysToRemove = new List<GridPoint>();
            foreach (var kvp in _occupiedGrids)
            {
                if (kvp.Value == robotId && kvp.Key != preserveGrid)
                {
                    occupiedKeysToRemove.Add(kvp.Key);
                }
            }
            foreach (var key in occupiedKeysToRemove)
            {
                _occupiedGrids.Remove(key);
            }

            _occupiedGrids[preserveGrid] = robotId;
        }

        public static long GetUndirectedEdgeKey(GridPoint u, GridPoint v)
        {
            int uCode = (u.X & 0xFFFF) | ((u.Y & 0xFFFF) << 16);
            int vCode = (v.X & 0xFFFF) | ((v.Y & 0xFFFF) << 16);
            long min = Math.Min(uCode, vCode);
            long max = Math.Max(uCode, vCode);
            return (min << 32) | (max & 0xFFFFFFFFL);
        }

        /// <summary>
        /// 检查目标栅格是否可被该机器人预约。
        /// </summary>
        public bool CanEnterGrid(int robotId, GridPoint targetGrid, out string conflictReason)
        {
            conflictReason = null;

            // 1. 检查是否有其他车正占领该栅格
            if (_occupiedGrids.TryGetValue(targetGrid, out int occupant) && occupant != robotId)
            {
                conflictReason = $"栅格 {targetGrid} 正被机器人 #{occupant} 实际占用";
                return false;
            }

            // 2. 检查是否有其他车已预约该栅格
            if (_reservedGrids.TryGetValue(targetGrid, out int reserver) && reserver != robotId)
            {
                conflictReason = $"栅格 {targetGrid} 已被机器人 #{reserver} 优先预约";
                return false;
            }

            return true;
        }

        /// <summary>
        /// 检查边是否可被该机器人通行 (防止对向通行冲突与同向追尾冲突)。
        /// </summary>
        public bool CanTraverseEdge(int robotId, GridPoint from, GridPoint to, out string conflictReason)
        {
            conflictReason = null;
            if (from == to) return true;

            long edgeKey = GetUndirectedEdgeKey(from, to);
            if (_reservedEdges.TryGetValue(edgeKey, out int edgeHolder) && edgeHolder != robotId)
            {
                conflictReason = $"边 ({from} <-> {to}) 正被机器人 #{edgeHolder} 占用/对向通行";
                return false;
            }

            return true;
        }

        /// <summary>
        /// 尝试为机器人申请从 from 到 to 的一步通行授权。
        /// </summary>
        public bool TryAcquireStepAuthorization(int robotId, GridPoint from, GridPoint to, out string failReason)
        {
            if (!CanEnterGrid(robotId, to, out failReason))
            {
                return false;
            }

            if (!CanTraverseEdge(robotId, from, to, out failReason))
            {
                return false;
            }

            // 授权成功，锁定目标栅格与边
            _reservedGrids[to] = robotId;
            if (from != to)
            {
                long edgeKey = GetUndirectedEdgeKey(from, to);
                _reservedEdges[edgeKey] = robotId;
            }

            _waitTimes[robotId] = 0f;
            return true;
        }

        /// <summary>
        /// 累加机器人等待时间，达到阈值时触发长等待告警。
        /// </summary>
        public void AccumulateWaitTime(int robotId, GridPoint blockedAt, float dt, string reason)
        {
            if (!_waitTimes.TryGetValue(robotId, out float t))
            {
                t = 0f;
            }
            t += dt;
            _waitTimes[robotId] = t;

            if (t >= LongWaitThresholdSeconds)
            {
                LongWaitDetected?.Invoke(robotId, blockedAt, t, reason);
            }
        }

        public float GetRobotWaitTime(int robotId)
        {
            _waitTimes.TryGetValue(robotId, out float t);
            return t;
        }

        /// <summary>
        /// 当机器人完全退出上一栅格、进入并稳定在目标栅格时，更新实际占用并释放原栅格和边资源。
        /// 仅凭逻辑判定不足，需结合机器人当前物理中心是否已离开原栅格范围。
        /// </summary>
        public void UpdateOccupancyAndRelease(
            int robotId,
            GridPoint lastConfirmedGrid,
            GridPoint targetGrid,
            GridPoint currentConfirmedGrid)
        {
            // 基于地标确认执行原子交接与双格安全锁释放。
            // lastConfirmedGrid: 机器人上一个已确认离开的地标栅格; currentConfirmedGrid: 机器人当前传感器扫描确认的地标栅格。
            if (currentConfirmedGrid != targetGrid)
            {
                // 未扫码确认对准目标地标，保持双格安全锁状态以防追尾
                return;
            }

            // 机器人确认进入目标栅格地标：原子化确立占用
            _occupiedGrids[targetGrid] = robotId;
            _reservedGrids.Remove(targetGrid);

            // 彻底释放上一地标与通道边占用
            if (lastConfirmedGrid != targetGrid)
            {
                if (_occupiedGrids.TryGetValue(lastConfirmedGrid, out int occupant) && occupant == robotId)
                {
                    _occupiedGrids.Remove(lastConfirmedGrid);
                }
                long edgeKey = GetUndirectedEdgeKey(lastConfirmedGrid, targetGrid);
                if (_reservedEdges.TryGetValue(edgeKey, out int edgeOwner) && edgeOwner == robotId)
                {
                    _reservedEdges.Remove(edgeKey);
                }
            }
        }

        public void SweepHistoricalReservations(int robotId, IReadOnlyList<GridPoint> path, GridPoint currentConfirmedGrid)
        {
            // 扫描并彻底释放机器人沿规划路径已经驶离且确认退出的所有历史栅格与通道边。
            // path: 机器人当前遵循的规划路径; currentConfirmedGrid: 机器人当前最新扫码确认所处的物理栅格。
            if (path == null || path.Count == 0) return;

            int confirmedIndex = -1;
            for (int i = 0; i < path.Count; i++)
            {
                if (path[i] == currentConfirmedGrid)
                {
                    confirmedIndex = i;
                    break;
                }
            }

            if (confirmedIndex <= 0) return;

            for (int i = 0; i < confirmedIndex; i++)
            {
                var pastGrid = path[i];
                if (pastGrid != currentConfirmedGrid)
                {
                    if (_occupiedGrids.TryGetValue(pastGrid, out int occupant) && occupant == robotId)
                    {
                        _occupiedGrids.Remove(pastGrid);
                    }
                    if (_reservedGrids.TryGetValue(pastGrid, out int reserver) && reserver == robotId)
                    {
                        _reservedGrids.Remove(pastGrid);
                    }
                }

                if (i < path.Count - 1)
                {
                    var nextGrid = path[i + 1];
                    long edgeKey = GetUndirectedEdgeKey(pastGrid, nextGrid);
                    if (_reservedEdges.TryGetValue(edgeKey, out int edgeOwner) && edgeOwner == robotId)
                    {
                        _reservedEdges.Remove(edgeKey);
                    }
                }
            }
        }

        public static List<GridPoint> GetOverlappingGrids(float x, float y, float radius, float cellSize)
        {
            // 计算指定世界坐标与包络半径在网格地图中发生几何交叠的全部栅格。
            // x: 世界横坐标; y: 世界纵坐标; radius: 包络圆半径; cellSize: 单网格物理边长。
            var result = new List<GridPoint>();
            if (cellSize <= 0f) return result;

            int minGx = (int)Math.Floor((x - radius) / cellSize);
            int maxGx = (int)Math.Floor((x + radius) / cellSize);
            int minGy = (int)Math.Floor((y - radius) / cellSize);
            int maxGy = (int)Math.Floor((y + radius) / cellSize);

            for (int gx = minGx; gx <= maxGx; gx++)
            {
                for (int gy = minGy; gy <= maxGy; gy++)
                {
                    float cellMinX = gx * cellSize;
                    float cellMaxX = (gx + 1) * cellSize;
                    float cellMinY = gy * cellSize;
                    float cellMaxY = (gy + 1) * cellSize;

                    float closestX = Math.Clamp(x, cellMinX, cellMaxX);
                    float closestY = Math.Clamp(y, cellMinY, cellMaxY);

                    float dx = x - closestX;
                    float dy = y - closestY;
                    if (dx * dx + dy * dy <= radius * radius)
                    {
                        result.Add(new GridPoint(gx, gy));
                    }
                }
            }

            return result;
        }

        public bool TryAcquireMultiGridOverlapLock(int robotId, IEnumerable<GridPoint> grids, out string failReason)
        {
            // 为偏位或处于田字格交界处的机器人原子化申请并锁定全部物理交叠栅格。
            // failReason: 申请交叠锁定失败时的冲突原因描述。
            failReason = null;
            if (grids == null) return true;

            foreach (var g in grids)
            {
                if (_occupiedGrids.TryGetValue(g, out int occupant) && occupant != robotId)
                {
                    failReason = $"交叠栅格 {g} 正被机器人 #{occupant} 占用";
                    return false;
                }
                if (_reservedGrids.TryGetValue(g, out int reserver) && reserver != robotId)
                {
                    failReason = $"交叠栅格 {g} 已被机器人 #{reserver} 预约";
                    return false;
                }
            }

            foreach (var g in grids)
            {
                _occupiedGrids[g] = robotId;
            }

            return true;
        }
    }
}
