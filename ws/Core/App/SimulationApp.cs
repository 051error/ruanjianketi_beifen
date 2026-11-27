using System;
using System.Collections.Generic;
using System.Linq;
using WarehouseGridSimulation.Core.Engine;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;
using WarehouseGridSimulation.Core.Planning;
using WarehouseGridSimulation.Core.Robots;

namespace WarehouseGridSimulation.Core.App
{
    /// <summary>
    /// 仿真应用入口外观类 (SimulationApp)。
    /// 作为 UI 交互宿主、测试用例与底层仿真引擎间的唯一权威交互界面。
    /// UI 只能通过该类发送控制意图与读取快照，严禁绕过该类直接篡改机器人位姿或预约状态。
    /// </summary>
    public class SimulationApp : IDisposable
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
            bool enableBg = Engine?.Config?.EnableBackgroundWorker ?? false;
            Engine?.Dispose();
            CurrentMapData = data;
            Engine = new SimulationEngine(map, new SimulationConfig { EnableBackgroundWorker = enableBg });
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

        public NavigationResult NavigateRobot(int robotId, GridPoint target, IPathPlanner customPlanner = null)
        {
            // 通过外观调度指定机器人执行导航移动。
            // robotId: 目标机器人编号; target: 目标网格坐标; customPlanner: 定制规划器。
            if (Engine == null) return NavigationResult.Fail("仿真引擎未就绪");
            return Engine.NavigateRobot(robotId, target, customPlanner);
        }

        public bool CancelNavigation(int robotId)
        {
            // 通过外观取消指定机器人的导航并执行安全制动。
            return Engine?.CancelNavigation(robotId) ?? false;
        }

        public MultiAgentPlanResult PlanMultiAgentPaths(IReadOnlyList<AgentTask> tasks, MultiAgentPlanOptions options = null)
        {
            // 通过外观门面调用多机器人协同路径规划算法。
            // tasks: 各智能体起终点任务列表; options: 可选的多机规划参数。
            if (Engine == null) return MultiAgentPlanResult.Fail("仿真引擎未就绪");
            return Engine.PlanMultiAgentPaths(tasks, options);
        }

        public bool ManualDriveRobot(int robotId, ManualDriveAction action, bool active)
        {
            // 向指定编号的机器人下发手动控制长按激活或释放制动信号。
            // robotId: 受控机器人全局编号; action: 手动驾驶动作类型; active: 控制信号是否处于按下激活状态。
            var robot = Engine?.GetRobot(robotId);
            if (robot == null) return false;

            if (active)
            {
                var mc = Engine.GetMotionController(robotId);
                mc?.ClearPath();
                Engine.ReservationManager.ReleaseRobotReservations(robotId, robot.ConfirmedGrid);

                switch (action)
                {
                    case ManualDriveAction.Forward:
                        if (Math.Abs(robot.AngularVelocity) <= 0.02f)
                            robot.ExecuteCommand(RobotCommand.MoveForward);
                        break;
                    case ManualDriveAction.TurnLeft:
                        if (robot.LinearVelocity <= robot.Config.SpeedTolerance)
                            robot.ExecuteCommand(RobotCommand.Rotate(-float.MaxValue));
                        break;
                    case ManualDriveAction.TurnRight:
                        if (robot.LinearVelocity <= robot.Config.SpeedTolerance)
                            robot.ExecuteCommand(RobotCommand.Rotate(float.MaxValue));
                        break;
                }

                if (!Engine.IsRunning || Engine.IsPaused) Engine.Start();
            }
            else
            {
                robot.ExecuteCommand(RobotCommand.Brake());
            }

            Engine.PublishSnapshot();
            return true;
        }

        public void MaintainManualDrive(int robotId, ManualDriveAction action)
        {
            // 在仿真时钟步进周期中维持指定机器人的长按手动动力学推进。
            var robot = Engine?.GetRobot(robotId);
            if (robot == null) return;

            var mc = Engine.GetMotionController(robotId);
            mc?.ClearPath();

            switch (action)
            {
                case ManualDriveAction.Forward:
                    if (Math.Abs(robot.AngularVelocity) <= 0.02f)
                        robot.ExecuteCommand(RobotCommand.MoveForward);
                    break;
                case ManualDriveAction.TurnLeft:
                    if (robot.LinearVelocity <= robot.Config.SpeedTolerance)
                        robot.ExecuteCommand(RobotCommand.Rotate(-float.MaxValue));
                    break;
                case ManualDriveAction.TurnRight:
                    if (robot.LinearVelocity <= robot.Config.SpeedTolerance)
                        robot.ExecuteCommand(RobotCommand.Rotate(float.MaxValue));
                    break;
            }

            if (!Engine.IsRunning || Engine.IsPaused) Engine.Start();
        }

        /// <summary>
        /// 无 UI 环境下的单步/批量仿真驱动入口，用于单元测试或控制台批处理。
        /// </summary>
        public SimulationSnapshot RunHeadless(int tickCount)
        {
            if (Engine == null) return null;

            SimulationSnapshot last = null;
            for (int i = 0; i < tickCount; i++)
            {
                last = Engine.StepSingleTick();
            }
            return last;
        }

        public void Dispose()
        {
            // 释放底层仿真引擎资源。
            Engine?.Dispose();
        }
    }

    /// <summary>
    /// 手动控制基础动作类型。
    /// </summary>
    public enum ManualDriveAction
    {
        Forward = 0,
        TurnLeft = 1,
        TurnRight = 2,
        Brake = 3
    }
}
