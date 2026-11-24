using WarehouseGridSimulation.Core.Geometry;

namespace WarehouseGridSimulation.Core.Map
{
    /// <summary>
    /// 仓储站点类型。
    /// </summary>
    public enum StationType
    {
        /// <summary>
        /// 上货点 / 取货点。
        /// </summary>
        Pick = 0,

        /// <summary>
        /// 卸货点 / 放货点。
        /// </summary>
        Drop = 1,

        /// <summary>
        /// 待命区 / 等待点。
        /// </summary>
        Waiting = 2
    }

    /// <summary>
    /// 仓储站点定义。
    /// </summary>
    public class Station
    {
        public int Id { get; set; }
        public string Name { get; set; }
        public StationType Type { get; set; }
        public GridPoint Position { get; set; }

        public Station() { }

        public Station(int id, string name, StationType type, GridPoint position)
        {
            Id = id;
            Name = name;
            Type = type;
            Position = position;
        }

        public override string ToString() => $"[Station #{Id} {Name} ({Type}) at {Position}]";
    }
}
