using System.Collections.Generic;

namespace WarehouseGridSimulation.Core.Map
{
    public class PointDto
    {
        public int X { get; set; }
        public int Y { get; set; }
    }

    public class StationDto
    {
        public int Id { get; set; }
        public string Name { get; set; }
        public string Type { get; set; } // "Pick", "Drop", "Waiting"
        public int X { get; set; }
        public int Y { get; set; }
    }

    public class RobotInitialDto
    {
        public int Id { get; set; }
        public int X { get; set; }
        public int Y { get; set; }
        public string Orientation { get; set; } // "Up", "Right", "Down", "Left"
    }

    /// <summary>
    /// 用于 JSON 序列化与反序列化的地图数据传输对象。
    /// </summary>
    public class MapData
    {
        public string Name { get; set; } = "WarehouseMap";
        public int Width { get; set; } = 30;
        public int Height { get; set; } = 30;
        public float CellSizeMeters { get; set; } = 0.55f;
        public List<PointDto> Obstacles { get; set; } = new List<PointDto>();
        public List<StationDto> Stations { get; set; } = new List<StationDto>();
        public List<RobotInitialDto> InitialRobots { get; set; } = new List<RobotInitialDto>();
    }
}
