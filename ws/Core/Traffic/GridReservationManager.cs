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
        // 机器人历史确认地标足迹 (用于三格移动闭塞延迟一格安全释放)
        private readonly Dictionary<int, List<GridPoint>> _confirmedFootprints = new Dictionary<int, List<GridPoint>>();

        public float LongWaitThresholdSeconds { get; set; } = 3.0f;

        /// <summary>
        /// 三格移动闭塞延迟释放模式 (默认在到达目标点后释放除当前目标点外的所有资源，作为当前多机调度的预约机制)。
        /// </summary>
        public ThreeGridDelayReleaseMode DelayReleaseMode { get; set; } = ThreeGridDelayReleaseMode.ReleaseAllExceptDestinationOnArrival;

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
            _confirmedFootprints.Clear();
        }

        public void RegisterRobotInitialGrid(int robotId, GridPoint grid)
        {
            _occupiedGrids[grid] = robotId;
            _waitTimes[robotId] = 0f;
            _confirmedFootprints[robotId] = new List<GridPoint> { grid };
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
            _confirmedFootprints[robotId] = new List<GridPoint> { preserveGrid };
        }

        public static long GetDirectedEdgeKey(GridPoint from, GridPoint to)
        {
            int fromCode = (from.X & 0xFFFF) | ((from.Y & 0xFFFF) << 16);
            int toCode = (to.X & 0xFFFF) | ((to.Y & 0xFFFF) << 16);
            return ((long)fromCode << 32) | (long)(uint)toCode;
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

            // 1. 迎面对穿坚决拦截 (对方正在逆向通行 to -> from)
            long reverseKey = GetDirectedEdgeKey(to, from);
            if (_reservedEdges.TryGetValue(reverseKey, out int oppHolder) && oppHolder != robotId)
            {
                conflictReason = $"边 ({from} <-> {to}) 正被机器人 #{oppHolder} 迎面逆向对穿占用";
                return false;
            }

            // 2. 兼容性无向边锁校验
            long undirectedKey = GetUndirectedEdgeKey(from, to);
            if (_reservedEdges.TryGetValue(undirectedKey, out int edgeHolder) && edgeHolder != robotId)
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

            // 授权成功，锁定目标栅格与有向通行边
            _reservedGrids[to] = robotId;
            if (from != to)
            {
                long directedKey = GetDirectedEdgeKey(from, to);
                _reservedEdges[directedKey] = robotId;
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

        public int? GetBlockingRobotId(int robotId, GridPoint from, GridPoint to)
        {
            // 查询阻碍指定机器人通行的目标栅格或通道边的占用者编号。
            // to: 目标栅格坐标。
            if (_occupiedGrids.TryGetValue(to, out int occupant) && occupant != robotId)
            {
                return occupant;
            }

            if (_reservedGrids.TryGetValue(to, out int reserver) && reserver != robotId)
            {
                return reserver;
            }

            if (from != to)
            {
                long reverseKey = GetDirectedEdgeKey(to, from);
                if (_reservedEdges.TryGetValue(reverseKey, out int oppHolder) && oppHolder != robotId)
                {
                    return oppHolder;
                }

                long undirectedKey = GetUndirectedEdgeKey(from, to);
                if (_reservedEdges.TryGetValue(undirectedKey, out int edgeHolder) && edgeHolder != robotId)
                {
                    return edgeHolder;
                }
            }

            return null;
        }

        /// <summary>
        /// 基于三格闭塞安全锁机制：小车扫码确认进入目标栅格时立项物理占用，推迟一格释放原栅格和边资源。
        /// 只有当小车进一步对准第二新地标 G_i 时，才释放 G_{i-2}，从而为小车尾部提供一整格身后的绝对安全空间。
        /// </summary>
        public void UpdateOccupancyAndRelease(
            int robotId,
            GridPoint lastConfirmedGrid,
            GridPoint targetGrid,
            GridPoint currentConfirmedGrid)
        {
            // 基于地标确认执行原子交接与三格移动闭塞延迟释放。
            // lastConfirmedGrid: 机器人上一个已确认的地标栅格; currentConfirmedGrid: 机器人当前传感器扫描确认的地标栅格。
            if (currentConfirmedGrid != targetGrid)
            {
                // 未扫码确认对准目标地标，保持多格闭塞安全锁状态以防追尾
                return;
            }

            // 机器人确认进入目标栅格地标：确立物理占用并移出预约
            _occupiedGrids[targetGrid] = robotId;
            _reservedGrids.Remove(targetGrid);

            if (!_confirmedFootprints.TryGetValue(robotId, out var footprints))
            {
                footprints = new List<GridPoint>();
                _confirmedFootprints[robotId] = footprints;
            }

            if (footprints.Count == 0 || footprints[^1] != targetGrid)
            {
                footprints.Add(targetGrid);
            }

            // 核心关键：延迟一格释放 (G_i 确认时释放 G_{i-2} 及其边锁，保留 G_i 与 G_{i-1} 物理占用)
            if (footprints.Count >= 3)
            {
                var twoStepsBehind = footprints[^3];
                var oneStepBehind = footprints[^2];

                if (_occupiedGrids.TryGetValue(twoStepsBehind, out int occupant) && occupant == robotId)
                {
                    _occupiedGrids.Remove(twoStepsBehind);
                }

                _reservedEdges.Remove(GetDirectedEdgeKey(twoStepsBehind, oneStepBehind));
                _reservedEdges.Remove(GetUndirectedEdgeKey(twoStepsBehind, oneStepBehind));
            }
        }

        public void ReleaseTrailOnDestinationReached(int robotId, GridPoint destinationGrid)
        {
            // 当机器人到达最终目的地并停稳时，清理先前滞留的身后旧地标占用与边锁，仅保留最终停车栅格。
            // destinationGrid: 机器人停靠的目标终点栅格。
            if (_confirmedFootprints.TryGetValue(robotId, out var footprints))
            {
                for (int i = 0; i < footprints.Count - 1; i++)
                {
                    var pastGrid = footprints[i];
                    if (pastGrid != destinationGrid)
                    {
                        if (_occupiedGrids.TryGetValue(pastGrid, out int occupant) && occupant == robotId)
                        {
                            _occupiedGrids.Remove(pastGrid);
                        }
                    }
                }

                _confirmedFootprints[robotId] = new List<GridPoint> { destinationGrid };
            }

            // 到达终点停稳后彻底清空所有通道边锁，杜绝滞留阻挡后车
            var edgeKeysToRemove = new List<long>();
            foreach (var kvp in _reservedEdges)
            {
                if (kvp.Value == robotId) edgeKeysToRemove.Add(kvp.Key);
            }
            foreach (var key in edgeKeysToRemove) _reservedEdges.Remove(key);

            _occupiedGrids[destinationGrid] = robotId;
        }

        public void OnDestinationReached(int robotId, GridPoint destinationGrid)
        {
            // 根据延迟释放模式处理机器人到达最终目标点后的占用资源与边锁释放。
            if (DelayReleaseMode == ThreeGridDelayReleaseMode.ReleaseAllExceptDestinationOnArrival)
            {
                ReleaseTrailOnDestinationReached(robotId, destinationGrid);
            }
            else
            {
                // LMAPF 模式：保留终点与身后上一格延迟保护，但彻底清空全部历史通道边锁与更早的栅格占用
                if (_confirmedFootprints.TryGetValue(robotId, out var footprints))
                {
                    if (footprints.Count >= 3)
                    {
                        for (int i = 0; i < footprints.Count - 2; i++)
                        {
                            var earlyGrid = footprints[i];
                            if (earlyGrid != destinationGrid)
                            {
                                if (_occupiedGrids.TryGetValue(earlyGrid, out int occupant) && occupant == robotId)
                                {
                                    _occupiedGrids.Remove(earlyGrid);
                                }
                            }
                        }
                        var prevGrid = footprints[^2];
                        _confirmedFootprints[robotId] = new List<GridPoint> { prevGrid, destinationGrid };
                    }
                }

                var edgeKeysToRemove = new List<long>();
                foreach (var kvp in _reservedEdges)
                {
                    if (kvp.Value == robotId) edgeKeysToRemove.Add(kvp.Key);
                }
                foreach (var key in edgeKeysToRemove) _reservedEdges.Remove(key);

                _occupiedGrids[destinationGrid] = robotId;
            }
        }

        public void SweepHistoricalReservations(int robotId, IReadOnlyList<GridPoint> path, GridPoint currentConfirmedGrid)
        {
            // 扫描并释放机器人沿规划路径更早前驶离的历史栅格与通道边，但保留身后上一格以维持三格闭塞安全锁。
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

            // 保持身后上一格安全占用，仅清扫 confirmedIndex - 2 及更早的历史栅格与边
            int sweepUntil = confirmedIndex - 1;
            if (sweepUntil <= 0) return;

            for (int i = 0; i < sweepUntil; i++)
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
                    _reservedEdges.Remove(GetDirectedEdgeKey(pastGrid, nextGrid));
                    _reservedEdges.Remove(GetUndirectedEdgeKey(pastGrid, nextGrid));
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
