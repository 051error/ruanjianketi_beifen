using System;
using System.Collections.Generic;
using System.Linq;
using WarehouseGridSimulation.Core.Geometry;

namespace WarehouseGridSimulation.Core.Map
{
    /// <summary>
    /// 二维栅格仓储地图模型。
    /// 坐标系说明：
    /// - 栅格坐标：X 为列索引 [0, Width - 1] (从左向右递增)；Y 为行索引 [0, Height - 1] (从上到下递增)。
    /// - 物理世界坐标：单位为米 (m)。每个栅格对应宽度和高度均为 CellSizeMeters 的正方形。
    /// - 栅格 (X, Y) 的物理中心为：((X + 0.5) * CellSizeMeters, (Y + 0.5) * CellSizeMeters)。
    /// </summary>
    public class GridMap
    {
        public int Width { get; }
        public int Height { get; }
        public float CellSizeMeters { get; }

        private readonly HashSet<GridPoint> _obstacles;
        private readonly Dictionary<int, Station> _stations;

        public IReadOnlyCollection<GridPoint> Obstacles => _obstacles;
        public IReadOnlyCollection<Station> Stations => _stations.Values;

        public IEnumerable<Station> PickStations => _stations.Values.Where(s => s.Type == StationType.Pick);
        public IEnumerable<Station> DropStations => _stations.Values.Where(s => s.Type == StationType.Drop);
        public IEnumerable<Station> WaitingStations => _stations.Values.Where(s => s.Type == StationType.Waiting);

        public GridMap(int width, int height, float cellSizeMeters = 0.55f)
        {
            if (width <= 0) throw new ArgumentOutOfRangeException(nameof(width), "地图宽度必须大于 0");
            if (height <= 0) throw new ArgumentOutOfRangeException(nameof(height), "地图高度必须大于 0");
            if (cellSizeMeters <= 0f) throw new ArgumentOutOfRangeException(nameof(cellSizeMeters), "网格边长必须大于 0");

            Width = width;
            Height = height;
            CellSizeMeters = cellSizeMeters;
            _obstacles = new HashSet<GridPoint>();
            _stations = new Dictionary<int, Station>();
        }

        public bool IsInside(GridPoint point)
        {
            return point.X >= 0 && point.X < Width && point.Y >= 0 && point.Y < Height;
        }

        public bool IsObstacle(GridPoint point)
        {
            return _obstacles.Contains(point);
        }

        public bool IsWalkable(GridPoint point)
        {
            return IsInside(point) && !IsObstacle(point);
        }

        public void SetObstacle(GridPoint point, bool isObstacle)
        {
            if (!IsInside(point)) return;
            if (isObstacle) _obstacles.Add(point);
            else _obstacles.Remove(point);
        }

        public void AddStation(Station station)
        {
            if (station == null) throw new ArgumentNullException(nameof(station));
            _stations[station.Id] = station;
        }

        public Station GetStation(int id)
        {
            _stations.TryGetValue(id, out var station);
            return station;
        }

        public (float X, float Y) GridToWorldCenter(GridPoint point)
        {
            return ((point.X + 0.5f) * CellSizeMeters, (point.Y + 0.5f) * CellSizeMeters);
        }

        public GridPoint WorldToGrid(float x, float y)
        {
            int gx = (int)Math.Floor(x / CellSizeMeters);
            int gy = (int)Math.Floor(y / CellSizeMeters);
            return new GridPoint(gx, gy);
        }
    }
}
