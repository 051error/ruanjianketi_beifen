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
            GridPoint currentGrid,
            GridPoint targetGrid,
            Pose2D currentPosition,
            float cellSizeMeters,
            float collisionRadius)
        {
            // 如果已到达目标栅格中心附近
            float targetCenterX = (targetGrid.X + 0.5f) * cellSizeMeters;
            float targetCenterY = (targetGrid.Y + 0.5f) * cellSizeMeters;
            float distToTargetCenter = currentPosition.DistanceTo(targetCenterX, targetCenterY);

            // 当机器人靠近目标栅格 (距离中心小于半个栅格边长) 且实际中心在 targetGrid 内
            int actualGridX = (int)Math.Floor(currentPosition.X / cellSizeMeters);
            int actualGridY = (int)Math.Floor(currentPosition.Y / cellSizeMeters);

            if (actualGridX == targetGrid.X && actualGridY == targetGrid.Y)
            {
                // 确认占有 targetGrid
                _occupiedGrids[targetGrid] = robotId;
                _reservedGrids.Remove(targetGrid);

                // 检查是否已经完全离开 currentGrid
                if (currentGrid != targetGrid)
                {
                    float oldCenterX = (currentGrid.X + 0.5f) * cellSizeMeters;
                    float oldCenterY = (currentGrid.Y + 0.5f) * cellSizeMeters;
                    float distToOldCenter = currentPosition.DistanceTo(oldCenterX, oldCenterY);

                    // 机器人边缘离老中心距离 > (半格宽度 + 安全余量)，确保物理彻底退出
                    if (distToOldCenter > cellSizeMeters * 0.7f)
                    {
                        if (_occupiedGrids.TryGetValue(currentGrid, out int occupant) && occupant == robotId)
                        {
                            _occupiedGrids.Remove(currentGrid);
                        }
                        long edgeKey = GetUndirectedEdgeKey(currentGrid, targetGrid);
                        if (_reservedEdges.TryGetValue(edgeKey, out int edgeOwner) && edgeOwner == robotId)
                        {
                            _reservedEdges.Remove(edgeKey);
                        }
                    }
                }
            }
        }
    }
}
