using System;
using System.Linq;
using WarehouseGridSimulation.Core.Engine;
using WarehouseGridSimulation.Core.Map;
using WarehouseGridSimulation.Core.Robots;

namespace WarehouseGridSimulation.Core.App
{
    /// <summary>
    /// 仿真应用入口外观类 (SimulationApp)。
    /// 作为 UI 交互宿主、测试用例与底层仿真引擎间的唯一权威交互界面。
    /// UI 只能通过该类发送控制意图与读取快照，严禁绕过该类直接篡改机器人位姿或预约状态。
    /// </summary>
    public class SimulationApp
    {
        public SimulationEngine Engine { get; private set; }
        public GridMap CurrentMap => Engine?.Map;
        public MapData CurrentMapData { get; private set; }

        public SimulationSnapshot CurrentSnapshot => Engine?.LatestSnapshot ?? Engine?.BuildSnapshot();

        public event Action<SimulationSnapshot> SnapshotUpdated;
        public event Action<string> LogEmitted;

        public SimulationApp()
        {
            LoadDefaultMap();
        }

        public void LoadDefaultMap()
        {
            var (map, data) = MapLoader.CreateDefaultWarehouseMap();
            InitializeEngineWithMap(map, data);
        }

        public void LoadMapFromJson(string json)
        {
            var (map, data) = MapLoader.LoadFromJson(json);
            InitializeEngineWithMap(map, data);
        }

        private void InitializeEngineWithMap(GridMap map, MapData data)
        {
            CurrentMapData = data;
            Engine = new SimulationEngine(map);
            Engine.SnapshotPublished += snapshot => SnapshotUpdated?.Invoke(snapshot);
            Engine.LogEmitted += log => LogEmitted?.Invoke(log);

            // 初始化配置机器人
            if (data.InitialRobots != null)
            {
                foreach (var rDto in data.InitialRobots)
                {
                    float heading = 0f;
                    if (string.Equals(rDto.Orientation, "Up", StringComparison.OrdinalIgnoreCase)) heading = -(float)Math.PI * 0.5f;
                    else if (string.Equals(rDto.Orientation, "Right", StringComparison.OrdinalIgnoreCase)) heading = 0f;
                    else if (string.Equals(rDto.Orientation, "Down", StringComparison.OrdinalIgnoreCase)) heading = (float)Math.PI * 0.5f;
                    else if (string.Equals(rDto.Orientation, "Left", StringComparison.OrdinalIgnoreCase)) heading = (float)Math.PI;

                    var robot = new SimulatedRobot(rDto.Id, new Geometry.GridPoint(rDto.X, rDto.Y), heading, map.CellSizeMeters);
                    Engine.AddRobot(robot);
                }
            }

            Engine.PublishSnapshot();
        }

        public void Start() => Engine?.Start();
        public void Pause() => Engine?.Pause();
        public void Resume() => Engine?.Resume();
        public void Step() => Engine?.StepSingleTick();

        public void Reset()
        {
            if (CurrentMapData != null)
            {
                var (map, data) = MapLoader.LoadFromJson(System.Text.Json.JsonSerializer.Serialize(CurrentMapData));
                InitializeEngineWithMap(map, data);
                Engine.Log("仿真已完全重置至 30x30 初始地图与单机器人位姿");
            }
            else
            {
                Engine?.Reset();
            }
        }

        public bool CreateTransportTask(int pickStationId, int dropStationId, int priority = 1)
        {
            if (Engine == null) return false;

            var pick = Engine.Map.GetStation(pickStationId);
            var drop = Engine.Map.GetStation(dropStationId);

            if (pick == null || drop == null)
            {
                Engine.Log($"[创建任务失败] 站点 ID 无效: Pick={pickStationId}, Drop={dropStationId}");
                return false;
            }

            var task = Engine.TaskManager.CreateTask(pick, drop, priority);
            Engine.Log($"创建搬运任务 #{task.Id}: [{pick.Name}] -> [{drop.Name}] (优先级 P{priority})");
            Engine.PublishSnapshot();

            return true;
        }

        public bool CreateRandomTask()
        {
            if (Engine == null) return false;
            var picks = Engine.Map.PickStations.ToList();
            var drops = Engine.Map.DropStations.ToList();

            if (picks.Count == 0 || drops.Count == 0) return false;

            var rnd = new Random();
            var pick = picks[rnd.Next(picks.Count)];
            var drop = drops[rnd.Next(drops.Count)];

            return CreateTransportTask(pick.Id, drop.Id, rnd.Next(1, 4));
        }

        /// <summary>
        /// 无 UI 环境下的单步/批量仿真驱动入口，用于单元测试或控制台批处理。
        /// </summary>
        public SimulationSnapshot RunHeadless(int tickCount)
        {
            if (Engine == null) return null;
            if (!Engine.IsRunning && !Engine.IsPaused)
            {
                Engine.Start();
            }

            SimulationSnapshot last = null;
            for (int i = 0; i < tickCount; i++)
            {
                last = Engine.StepSingleTick();
            }
            return last;
        }
    }
}
