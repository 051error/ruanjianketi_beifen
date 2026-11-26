using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using WarehouseGridSimulation.Core.App;
using WarehouseGridSimulation.Core.Collision;
using WarehouseGridSimulation.Core.Engine;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;
using WarehouseGridSimulation.Core.Motion;
using WarehouseGridSimulation.Core.Planning;
using WarehouseGridSimulation.Core.Robots;
using WarehouseGridSimulation.Core.Tasks;
using WarehouseGridSimulation.Core.Traffic;
using TaskStatus = WarehouseGridSimulation.Core.Tasks.TaskStatus;

namespace WarehouseGridSimulation.Tests
{
    [TestClass]
    public class WarehouseGridSimulationTests
    {
        [TestMethod]
        public void PoseInterpolation_UsesShortestAngleAndClampsEndpoints()
        {
            // 验证位姿插值跨越角度边界并限制插值范围。
            var from = new Pose2D(1f, 2f, 170f * (float)Math.PI / 180f);
            var to = new Pose2D(3f, 6f, -170f * (float)Math.PI / 180f);

            var middle = Pose2D.Lerp(from, to, 0.5f);
            Assert.AreEqual(2f, middle.X, 1e-5f);
            Assert.AreEqual(4f, middle.Y, 1e-5f);
            Assert.AreEqual((float)Math.PI, Math.Abs(middle.Heading), 1e-5f);
            Assert.AreEqual(from, Pose2D.Lerp(from, to, -1f));
            Assert.AreEqual(to, Pose2D.Lerp(from, to, 2f));

            var reverse = Pose2D.Lerp(to, from, 0.5f);
            Assert.AreEqual((float)Math.PI, Math.Abs(reverse.Heading), 1e-5f);
        }

        [TestMethod]
        public void Snapshot_PreviousPositionTracksStepsAndResetWithoutMutatingPublishedSnapshots()
        {
            // 验证快照保留上一步位姿并在复位后立即同步。
            var map = new GridMap(10, 10, 0.55f);
            var engine = new SimulationEngine(map, new SimulationConfig { AutoPauseOnCollision = false });
            var robot = new SimulatedRobot(1, new GridPoint(1, 1), 0f, map.CellSizeMeters);
            engine.AddRobot(robot);
            robot.ExecuteCommand(RobotCommand.Move(1f));
            engine.Start();

            var first = engine.Step();
            var firstRobot = first.Robots[0];
            var firstPosition = firstRobot.Position;
            var firstPreviousPosition = firstRobot.PreviousPosition;
            Assert.AreEqual(robot.PreviousPosition, firstRobot.PreviousPosition);
            Assert.IsTrue(firstRobot.Position.X > firstRobot.PreviousPosition.X);

            var second = engine.Step();
            Assert.AreEqual(firstPosition, second.Robots[0].PreviousPosition);
            Assert.AreEqual(firstPreviousPosition, first.Robots[0].PreviousPosition);

            var resetPose = new Pose2D(2f, 2f, 1f);
            robot.TeleportForReset(resetPose);
            var reset = engine.PublishSnapshot();
            Assert.AreEqual(resetPose, reset.Robots[0].Position);
            Assert.AreEqual(resetPose, reset.Robots[0].PreviousPosition);
            Assert.AreEqual(firstPosition, first.Robots[0].Position);
        }

        // 场景 1：JSON 地图加载及非法地图校验
        [TestMethod]
        public void Scenario1_JsonMapLoadingAndValidation()
        {
            // 1.1 合法地图加载验证
            string validJson = @"{
                ""name"": ""TestMap"",
                ""width"": 10,
                ""height"": 10,
                ""cellSizeMeters"": 0.55,
                ""obstacles"": [ { ""x"": 3, ""y"": 3 } ],
                ""stations"": [
                    { ""id"": 1, ""name"": ""PickStation"", ""type"": ""Pick"", ""x"": 1, ""y"": 1 },
                    { ""id"": 2, ""name"": ""DropStation"", ""type"": ""Drop"", ""x"": 8, ""y"": 8 }
                ],
                ""initialRobots"": [
                    { ""id"": 1, ""x"": 0, ""y"": 0, ""orientation"": ""Right"" },
                    { ""id"": 2, ""x"": 0, ""y"": 1, ""orientation"": ""Right"" }
                ]
            }";

            var (map, data) = MapLoader.LoadFromJson(validJson);
            Assert.IsNotNull(map);
            Assert.AreEqual(10, map.Width);
            Assert.AreEqual(10, map.Height);
            Assert.IsTrue(map.IsObstacle(new GridPoint(3, 3)));
            Assert.AreEqual(2, map.Stations.Count);
            Assert.AreEqual(2, data.InitialRobots.Count);

            // 1.2 非法地图校验：越界与重叠校验
            var invalidData = new MapData
            {
                Width = 10,
                Height = 10,
                CellSizeMeters = 0.55f,
                Obstacles = new List<PointDto> { new PointDto { X = 15, Y = 15 } }, // 越界障碍物
                Stations = new List<StationDto> { new StationDto { Id = 1, Name = "St1", Type = "Pick", X = 20, Y = 20 } }, // 越界站点
                InitialRobots = new List<RobotInitialDto>
                {
                    new RobotInitialDto { Id = 1, X = 2, Y = 2, Orientation = "Right" },
                    new RobotInitialDto { Id = 2, X = 2, Y = 2, Orientation = "Right" } // 坐标重叠
                }
            };

            bool isValid = MapLoader.Validate(invalidData, out var errors);
            Assert.IsFalse(isValid);
            Assert.IsTrue(errors.Any(e => e.Contains("障碍物坐标越界")));
            Assert.IsTrue(errors.Any(e => e.Contains("坐标越界")));
            Assert.IsTrue(errors.Any(e => e.Contains("重叠")));

            // 尺寸非法独立校验
            var invalidSizeData = new MapData { Width = -5, Height = 10 };
            Assert.IsFalse(MapLoader.Validate(invalidSizeData, out var sizeErrors));
            Assert.IsTrue(sizeErrors.Any(e => e.Contains("尺寸非法")));
        }

        // 场景 2：A* 找到可行路径；不可达时返回明确结果
        [TestMethod]
        public void Scenario2_AStarPathFindingAndUnreachable()
        {
            var map = new GridMap(10, 10, 0.55f);
            var planner = new AStarPlanner();

            // 2.1 正常寻路
            var start = new GridPoint(1, 1);
            var goal = new GridPoint(1, 5);
            var result = planner.PlanPath(map, start, goal);

            Assert.IsTrue(result.Success);
            Assert.IsNotNull(result.Path);
            Assert.AreEqual(5, result.Path.Count);
            Assert.AreEqual(start, result.Path[0]);
            Assert.AreEqual(goal, result.Path[^1]);

            // 2.2 不可达寻路 (用障碍物将目标死死围住)
            var trappedGoal = new GridPoint(8, 8);
            map.SetObstacle(new GridPoint(7, 8), true);
            map.SetObstacle(new GridPoint(9, 8), true);
            map.SetObstacle(new GridPoint(8, 7), true);
            map.SetObstacle(new GridPoint(8, 9), true);

            var unreachableResult = planner.PlanPath(map, start, trappedGoal);
            Assert.IsFalse(unreachableResult.Success);
            Assert.IsTrue(unreachableResult.ErrorMessage.Contains("未找到") || unreachableResult.ErrorMessage.Contains("不可达"));
        }

        // 场景 3：单车完成“前往上货点 → 等待上货 → 前往卸货点 → 等待卸货 → 完成”
        [TestMethod]
        public void Scenario3_SingleRobotFullTaskCycle()
        {
            var map = new GridMap(10, 10, 0.55f);
            var pick = new Station(1, "Pick-1", StationType.Pick, new GridPoint(2, 2));
            var drop = new Station(2, "Drop-1", StationType.Drop, new GridPoint(4, 2));
            map.AddStation(pick);
            map.AddStation(drop);

            var engine = new SimulationEngine(map, new SimulationConfig { FixedDeltaTime = 0.1f });
            var robot = new SimulatedRobot(1, new GridPoint(1, 2), 0f, 0.55f);
            engine.AddRobot(robot);

            var task = engine.TaskManager.CreateTask(pick, drop);
            task.LoadingTimeSeconds = 0.2f;   // 模拟上货耗时 0.2s
            task.UnloadingTimeSeconds = 0.2f; // 模拟卸货耗时 0.2s

            engine.Start();

            // 驱动仿真直到任务完成或超时
            int maxTicks = 200;
            for (int i = 0; i < maxTicks && task.Status != TaskStatus.Completed; i++)
            {
                engine.Step();
            }

            Assert.AreEqual(TaskStatus.Completed, task.Status, $"任务应顺利完成，当前状态为: {task.Status}");
            Assert.IsFalse(robot.HasCargo, "卸货完成后机器人应不再载货");
            Assert.IsNull(robot.CurrentTaskId, "任务完成后机器人应恢复空闲待命");
        }

        // 场景 4：两车不能同时进入同一栅格
        [TestMethod]
        public void Scenario4_TwoRobotsCannotEnterSameGrid()
        {
            var manager = new GridReservationManager();
            var targetGrid = new GridPoint(5, 5);

            manager.RegisterRobotInitialGrid(1, new GridPoint(4, 5));
            manager.RegisterRobotInitialGrid(2, new GridPoint(6, 5));

            // 机器人 1 成功申请进入 targetGrid
            bool r1Allowed = manager.TryAcquireStepAuthorization(1, new GridPoint(4, 5), targetGrid, out string reason1);
            Assert.IsTrue(r1Allowed);
            Assert.IsNull(reason1);

            // 机器人 2 尝试进入已被机器人 1 预约的 targetGrid，必须被严正拒绝
            bool r2Allowed = manager.TryAcquireStepAuthorization(2, new GridPoint(6, 5), targetGrid, out string reason2);
            Assert.IsFalse(r2Allowed);
            Assert.IsNotNull(reason2);
            Assert.IsTrue(reason2.Contains("优先预约") || reason2.Contains("占用"));
        }

        // 场景 5：两车不能在同一条边上对向通过
        [TestMethod]
        public void Scenario5_TwoRobotsCannotCrossSameEdgeOpposite()
        {
            var manager = new GridReservationManager();
            var gridA = new GridPoint(3, 3);
            var gridB = new GridPoint(4, 3);
            var gridC = new GridPoint(5, 3);

            manager.RegisterRobotInitialGrid(1, gridA);
            manager.RegisterRobotInitialGrid(2, gridC);

            // 机器人 1 申请从 A 到 B 的边通行权限
            bool r1Allowed = manager.TryAcquireStepAuthorization(1, gridA, gridB, out string r1Fail);
            Assert.IsTrue(r1Allowed, $"R1 应成功获得通行授权: {r1Fail}");

            // 机器人 2 申请从 B 到 A 的对向边通行权限，必须被阻止以杜绝 Head-on 撞车
            bool r2Allowed = manager.TryAcquireStepAuthorization(2, gridB, gridA, out string reason);
            Assert.IsFalse(r2Allowed, "对向申请同一条边应被绝对拒绝");
            Assert.IsNotNull(reason);
            Assert.IsTrue(reason.Contains("对向通行") || reason.Contains("占用"));
        }

        // 场景 6：机器人到达目标后能够停止，并满足设定容差
        [TestMethod]
        public void Scenario6_RobotStopsAtTargetWithinTolerance()
        {
            float cellSize = 0.55f;
            var robot = new SimulatedRobot(1, new GridPoint(0, 0), 0f, cellSize);
            var controller = new MotionController(1);

            var path = new List<GridPoint> { new GridPoint(0, 0), new GridPoint(1, 0) };
            controller.SetPath(path, initialAuthorizedIndex: 1);

            float dt = 0.05f;
            int maxSteps = 150;

            for (int i = 0; i < maxSteps && !controller.IsPathCompleted; i++)
            {
                var cmd = controller.ComputeNextCommand(robot, cellSize, dt);
                robot.ExecuteCommand(cmd);
                robot.Step(dt, cellSize);
            }

            Assert.IsTrue(controller.IsPathCompleted, "运动控制器应判定路径到达完成");

            // 验证停靠位置容差与速度容差
            float targetPhysX = (1 + 0.5f) * cellSize;
            float targetPhysY = (0 + 0.5f) * cellSize;
            float dist = robot.Position.DistanceTo(targetPhysX, targetPhysY);

            Assert.IsTrue(dist <= robot.Config.PositionTolerance, $"停靠位置误差 {dist:F3}m 超出容差 {robot.Config.PositionTolerance:F3}m");
            Assert.IsTrue(robot.LinearVelocity <= robot.Config.SpeedTolerance, $"停靠后线速度 {robot.LinearVelocity:F3}m/s 超出容差");
        }

        // 场景 7：暂停后仿真时间和机器人位置不再推进；单步只推进一个 Tick
        [TestMethod]
        public void Scenario7_PauseStopsAdvancement_StepAdvancesSingleTick()
        {
            var map = new GridMap(10, 10, 0.55f);
            var engine = new SimulationEngine(map, new SimulationConfig { FixedDeltaTime = 0.1f });
            var robot = new SimulatedRobot(1, new GridPoint(1, 1), 0f, 0.55f);
            engine.AddRobot(robot);

            engine.Start();
            engine.Step();
            float timeAfterStart = engine.SimulationTime;
            long tickAfterStart = engine.Tick;

            // 7.1 暂停
            engine.Pause();
            engine.Step(); // 在暂停状态调用常规 Step
            Assert.AreEqual(timeAfterStart, engine.SimulationTime, "暂停后常规 Step 不应推进仿真时间");
            Assert.AreEqual(tickAfterStart, engine.Tick, "暂停后常规 Step 不应推进 Tick");

            // 7.2 单步推进 (StepSingleTick)
            engine.StepSingleTick();
            Assert.AreEqual(timeAfterStart + 0.1f, engine.SimulationTime, 1e-4, "单步应精确推进一个 FixedDeltaTime");
            Assert.AreEqual(tickAfterStart + 1, engine.Tick, "单步应精确增加 1 个 Tick");
        }

        // 场景 8：无 UI 环境下可运行仿真核心
        [TestMethod]
        public void Scenario8_HeadlessExecutionWithoutUi()
        {
            var app = new SimulationApp();
            Assert.IsNotNull(app.CurrentMap);

            // 无任何窗口或 UI 控件环境下执行 30 步仿真
            var snapshot = app.RunHeadless(30);

            Assert.IsNotNull(snapshot);
            Assert.AreEqual(30, snapshot.Tick);
            Assert.IsTrue(snapshot.SimulationTime > 0f);
            Assert.IsNotNull(snapshot.Robots);
            Assert.IsTrue(snapshot.Robots.Count > 0);
        }

        // 场景 9：独立碰撞检测能发现至少一个构造出的碰撞场景
        [TestMethod]
        public void Scenario9_IndependentCollisionDetector_DetectsCollision()
        {
            var detector = new CollisionDetector();
            var map = new GridMap(10, 10, 0.55f);

            // 构造两台机器人中心间距 0.20m (小于两车半径之和 0.25 + 0.25 = 0.50m) 的危险碰撞位姿
            var robotA = new SimulatedRobot(1, 2.0f, 2.0f, 0f, 0.55f);
            var robotB = new SimulatedRobot(2, 2.2f, 2.0f, (float)Math.PI, 0.55f);

            var collisions = detector.CheckCollisions(1.0f, new[] { robotA, robotB }, map);

            Assert.IsTrue(collisions.Count > 0, "独立碰撞检测器应成功发现车身重叠碰撞");
            Assert.AreEqual(CollisionType.RobotWithRobot, collisions[0].Type);
            Assert.AreEqual(1, collisions[0].RobotId1);
            Assert.AreEqual(2, collisions[0].RobotId2);
        }

        // 场景 10：验证 SimulationApp 启动、暂停、继续、单步、重置与添加任务的响应性
        [TestMethod]
        public void Scenario10_AppControlApiAndTaskCreation()
        {
            var app = new SimulationApp();

            // 1. 验证启动指令生效
            app.Start();
            Assert.IsTrue(app.Engine.IsRunning, "调用 Start 后引擎应处于 Running 状态");
            Assert.IsFalse(app.Engine.IsPaused, "调用 Start 后引擎不应处于 Paused 状态");

            // 2. 验证暂停与恢复
            app.Pause();
            Assert.IsTrue(app.Engine.IsPaused, "调用 Pause 后引擎应处于 Paused 状态");

            app.Resume();
            Assert.IsFalse(app.Engine.IsPaused, "调用 Resume 后引擎应恢复非 Paused 状态");

            // 3. 验证任务创建
            int taskCountBefore = app.Engine.TaskManager.Tasks.Count;
            bool created = app.CreateRandomTask();
            Assert.IsTrue(created, "应成功创建随机运输任务");
            Assert.AreEqual(taskCountBefore + 1, app.Engine.TaskManager.Tasks.Count, "任务列表应立即增加新任务");

            // 4. 验证单步与重置
            long tickBefore = app.Engine.Tick;
            app.Step();
            Assert.AreEqual(tickBefore + 1, app.Engine.Tick, "调用 Step 后 Tick 应立即加 1");

            app.Reset();
            Assert.IsFalse(app.Engine.IsRunning, "调用 Reset 后引擎应重置为未启动状态");
            Assert.AreEqual(0, app.Engine.Tick, "调用 Reset 后 Tick 应重置为 0");
        }

        // 场景 11：验证机器人最大速度 1.5m/s、加速度 1.5m/s^2、角速度 180°/s、角加速度 360°/s^2 与中心读码约束
        [TestMethod]
        public void Scenario11_KinematicsParametersAndCenterGridPerception()
        {
            float cellSize = 0.55f;
            var robot = new SimulatedRobot(1, new GridPoint(0, 0), 0f, cellSize);

            // 1. 验证默认动力学参数
            Assert.AreEqual(1.5f, robot.Config.MaxLinearSpeed, 1e-4);
            Assert.AreEqual(1.5f, robot.Config.LinearAcceleration, 1e-4);
            Assert.AreEqual((float)Math.PI, robot.Config.TurnSpeed, 1e-4);
            Assert.AreEqual(2f * (float)Math.PI, robot.Config.AngularAcceleration, 1e-4);

            // 2. 验证角加速度动力学推进 (角速度平滑加减速，非瞬间突变)
            robot.ExecuteCommand(RobotCommand.TurnRight);
            robot.Step(0.1f, cellSize);
            Assert.IsTrue(robot.AngularVelocity > 0f);
            Assert.IsTrue(robot.AngularVelocity < robot.Config.TurnSpeed, "角速度应随角加速度平滑累加，不可瞬间突变");

            // 3. 验证中心读码约束：只有移动到栅格正中央才更新所在栅格
            var robotMover = new SimulatedRobot(2, new GridPoint(0, 0), 0f, cellSize);
            Assert.AreEqual(new GridPoint(0, 0), robotMover.GridPosition);

            // 往前移动一段距离，使其物理跨过栅格边界 (但尚未到达网格 1 的正中心 0.825m)
            robotMover.ExecuteCommand(RobotCommand.MoveForward);
            for (int i = 0; i < 5; i++)
            {
                robotMover.Step(0.08f, cellSize);
            }

            float center1X = (1 + 0.5f) * cellSize;
            float distToCenter1 = Math.Abs(robotMover.Position.X - center1X);
            if (distToCenter1 > robotMover.Config.PositionTolerance)
            {
                // 未达网格 1 中心，内部感知依然确认为 (0, 0)
                Assert.AreEqual(new GridPoint(0, 0), robotMover.GridPosition, "未到达栅格正中央前不可判定为新栅格");
            }

            // 继续前进直到到达栅格 1 正中央
            while (Math.Abs(robotMover.Position.X - center1X) > robotMover.Config.PositionTolerance && robotMover.Position.X < center1X)
            {
                robotMover.Step(0.01f, cellSize);
            }
            Assert.AreEqual(new GridPoint(1, 0), robotMover.GridPosition, "到达栅格正中央后方可识别并更新为当前栅格");
        }

        // 场景 12：验证前进位移、转向角度、停止制动信号及互斥约束
        [TestMethod]
        public void Scenario12_SignalTypesAndContinuousMotionWithMutualExclusion()
        {
            float cellSize = 0.55f;
            var robot = new SimulatedRobot(1, new GridPoint(5, 5), 0f, cellSize);

            // 1. 验证定量前进位移信号 (前进 1.10 米并停止)
            robot.ExecuteCommand(RobotCommand.Move(1.10f));
            for (int i = 0; i < 40; i++)
            {
                robot.Step(0.05f, cellSize);
            }
            Assert.AreEqual(0f, robot.LinearVelocity, 1e-4, "定量位移执行完毕后应自动减速制动停稳");
            float startCenterX = (5 + 0.5f) * cellSize;
            float movedDist = robot.Position.X - startCenterX;
            Assert.AreEqual(1.10f, movedDist, 0.05f, "前进距离应精确匹配设定位移参数");

            // 2. 验证定量自旋角度信号 (原地顺时针旋转 90 度)
            robot.ExecuteCommand(RobotCommand.Rotate((float)Math.PI * 0.5f));
            for (int i = 0; i < 40; i++)
            {
                robot.Step(0.05f, cellSize);
            }
            Assert.AreEqual(0f, robot.AngularVelocity, 1e-4, "定量旋转执行完毕后角速度应归零停稳");
            Assert.AreEqual((float)Math.PI * 0.5f, robot.Position.Heading, 0.05f, "最终航向应匹配设定旋转角度");

            // 3. 验证转向与前进严格互斥 (不能边转边走)
            // 3.1 前进中尝试转向被安全拒绝
            robot.ExecuteCommand(RobotCommand.Move(10f));
            robot.Step(0.1f, cellSize);
            Assert.IsTrue(robot.LinearVelocity > 0.1f, "机器人正在前进加速");
            robot.ExecuteCommand(RobotCommand.Rotate((float)Math.PI * 0.5f));
            Assert.AreEqual(0f, robot.RemainingAngle, "前进运动中下发转向角度应被严格拒绝，剩余旋转角为 0");
            Assert.AreEqual(0f, robot.AngularVelocity, "前进运动中角速度应强制保持为 0");

            // 3.2 停止制动滑行至完全静止 (方案 A)
            robot.ExecuteCommand(RobotCommand.Stop);
            while (robot.LinearVelocity > 0f)
            {
                robot.Step(0.05f, cellSize);
            }
            Assert.AreEqual(0f, robot.LinearVelocity, "停止制动后线速度应完全降为 0");

            // 3.3 原地旋转中尝试前进被安全拒绝
            robot.ExecuteCommand(RobotCommand.Rotate((float)Math.PI * 0.5f));
            robot.Step(0.1f, cellSize);
            Assert.IsTrue(Math.Abs(robot.AngularVelocity) > 0.1f, "机器人正在原地旋转");
            robot.ExecuteCommand(RobotCommand.Move(5f));
            Assert.AreEqual(0f, robot.RemainingDistance, "原地旋转中下发前进位移应被严格拒绝，剩余位移为 0");
            Assert.AreEqual(0f, robot.LinearVelocity, "原地旋转中线速度应强制保持为 0");
        }

        // 场景 13：Dijkstra 规划器基础路径搜索与不可达防护
        [TestMethod]
        public void Scenario13_DijkstraPathFindingAndUnreachable()
        {
            // 验证 Dijkstra 规划器能找到合法最短路径并在死路时安全报错。
            // dijkstra: 待测试的 Dijkstra 算法规划器实例; map: 仿真地图模型; start: 起始网格坐标; goal: 目标网格坐标; trappedGoal: 被障碍物围困的目标坐标。
            var map = new GridMap(10, 10, 0.55f);
            var dijkstra = new DijkstraPlanner();

            var start = new GridPoint(1, 1);
            var goal = new GridPoint(1, 6);
            var result = dijkstra.PlanPath(map, start, goal);

            Assert.IsTrue(result.Success, "Dijkstra 应成功规划可行路径");
            Assert.AreEqual(6, result.Path.Count);
            Assert.AreEqual(start, result.Path[0]);
            Assert.AreEqual(goal, result.Path[^1]);

            // 测试障碍物阻挡不可达场景
            var trappedGoal = new GridPoint(8, 8);
            map.SetObstacle(new GridPoint(7, 8), true);
            map.SetObstacle(new GridPoint(9, 8), true);
            map.SetObstacle(new GridPoint(8, 7), true);
            map.SetObstacle(new GridPoint(8, 9), true);

            var trappedResult = dijkstra.PlanPath(map, start, trappedGoal);
            Assert.IsFalse(trappedResult.Success, "目标被死锁围困时 Dijkstra 应返回规划失败");
        }

        // 场景 14：A* 算法在 0.3 曼哈顿距离 + 0.7 转向成本启发式下规划出最小转弯次数路径
        [TestMethod]
        public void Scenario14_AStarTurnCostHeuristicMinimizesTurns()
        {
            // 验证 A* 在对角目标场景下优先选择单次折线的少拐弯路径。
            // turns: 规划路径中转向节点的累计次数; dx1: 前一段路径在水平轴的位移增量; dy1: 前一段路径在垂直轴的位移增量; dx2: 后一段路径在水平轴的位移增量; dy2: 后一段路径在垂直轴的位移增量。
            var map = new GridMap(10, 10, 0.55f);
            var planner = new AStarPlanner();

            var start = new GridPoint(1, 1);
            var goal = new GridPoint(5, 5);
            var result = planner.PlanPath(map, start, goal);

            Assert.IsTrue(result.Success, "A* 应成功找到可行路径");
            Assert.AreEqual(start, result.Path[0]);
            Assert.AreEqual(goal, result.Path[^1]);

            // 统计路径中的转弯次数
            int turns = 0;
            for (int i = 1; i < result.Path.Count - 1; i++)
            {
                int dx1 = result.Path[i].X - result.Path[i - 1].X;
                int dy1 = result.Path[i].Y - result.Path[i - 1].Y;
                int dx2 = result.Path[i + 1].X - result.Path[i].X;
                int dy2 = result.Path[i + 1].Y - result.Path[i].Y;
                if (dx1 != dx2 || dy1 != dy2)
                {
                    turns++;
                }
            }

            Assert.AreEqual(1, turns, "在开阔网格对角导航场景下，带转向成本启发式的 A* 应规划出仅有 1 次拐弯的最优路线");
        }

        // 场景 15：无 UI 环境下通过 SimulationApp.NavigateRobot 调度单机导航与自动推进闭环
        [TestMethod]
        public void Scenario15_HeadlessNavigateRobotAndArrival()
        {
            // 验证核心层单机导航外观接口在无界面环境下的寻路、下发与步进完成闭环。
            // app: 仿真应用外观实例; target: 导航目标网格; navResult: 导航启动结果对象; mc: 机器人关联运动控制器。
            var app = new SimulationApp();
            var target = new GridPoint(14, 12); // 朝向上方直行两格

            var navResult = app.NavigateRobot(1, target);
            Assert.IsTrue(navResult.Success, "无 UI 调度单机导航应成功规划并启动");
            Assert.IsTrue(navResult.Path.Count > 1, "规划生成的路径应包含多个节点");
            Assert.IsTrue(app.Engine.IsRunning, "下发导航路径后仿真引擎应自动处于运行状态");

            var mc = app.Engine.GetMotionController(1);
            Assert.IsNotNull(mc, "运动控制器应成功绑定");
            Assert.IsTrue(mc.HasPath, "运动控制器应接收到规划路径");

            // 后台驱动推进 40 步 (4 秒，足够直行加速、匀速巡航与梯形平稳减速停稳)
            app.RunHeadless(40);

            var r = app.Engine.GetRobot(1);
            Assert.IsNotNull(r);
            Assert.AreEqual(target, r.ConfirmedGrid, "机器人应在仿真步长推进后精确到达目标网格中心");
            Assert.IsTrue(mc.IsPathCompleted, "到达目标后控制器路径完成标志应为 true");
            Assert.IsTrue(r.LinearVelocity <= r.Config.SpeedTolerance, "到达后线速度应满足停靠容差");
        }

        // 场景 16：无 UI 环境下取消导航制动与非法目标边界防护
        [TestMethod]
        public void Scenario16_HeadlessCancelNavigationAndBoundaryGuards()
        {
            // 验证取消导航制动以及越界、障碍物与已在原地时的边界防御。
            // outOfBoundsTarget: 越界目标网格; obstacleTarget: 障碍物目标网格; sameTarget: 原地目标网格; cancelOk: 取消导航执行是否成功。
            var app = new SimulationApp();

            // 1. 越界目标防护
            var outOfBoundsTarget = new GridPoint(100, 100);
            var result1 = app.NavigateRobot(1, outOfBoundsTarget);
            Assert.IsFalse(result1.Success, "越界目标应被安全拒绝");
            Assert.IsTrue(result1.Message.Contains("超出地图范围"));

            // 2. 障碍物目标防护
            var obstacleTarget = new GridPoint(14, 18);
            app.CurrentMap.SetObstacle(obstacleTarget, true);
            var result2 = app.NavigateRobot(1, obstacleTarget);
            Assert.IsFalse(result2.Success, "位于障碍物内的目标应被安全拒绝");
            Assert.IsTrue(result2.Message.Contains("障碍物"));

            // 3. 当前已在目标点网格防护
            var r = app.Engine.GetRobot(1);
            var sameTarget = r.ConfirmedGrid;
            var result3 = app.NavigateRobot(1, sameTarget);
            Assert.IsFalse(result3.Success, "已在目标点时应提示已处于目标网格");
            Assert.IsTrue(result3.Message.Contains("已处于目标点网格"));

            // 4. 正常下发后中途取消导航并安全制动
            var validTarget = new GridPoint(14, 20);
            var result4 = app.NavigateRobot(1, validTarget);
            Assert.IsTrue(result4.Success);

            app.RunHeadless(5); // 移动若干步
            var cancelOk = app.CancelNavigation(1);
            Assert.IsTrue(cancelOk, "取消导航应执行成功");

            var mc = app.Engine.GetMotionController(1);
            Assert.IsTrue(!mc.HasPath || mc.IsPathCompleted, "取消后控制器路径应已被清空");
        }

        // 场景 17：大跨度开阔对角场景规划严格为单次拐弯的标准 L 型路径
        [TestMethod]
        public void Scenario17_LongDiagonalPathIsStrictLShaped()
        {
            // 验证带朝向状态与转向惩罚的 A* 在大跨度对角网格下规划出拐弯次数严格为 1 的标准 L 型路径。
            // map: 30x30 开阔网格地图; planner: 待测 A* 规划器; start: 起点坐标; goal: 对角目标点坐标; turns: 实际统计转弯次数。
            var map = new GridMap(30, 30, 0.55f);
            var planner = new AStarPlanner();

            var start = new GridPoint(2, 2);
            var goal = new GridPoint(25, 20);
            var result = planner.PlanPath(map, start, goal);

            Assert.IsTrue(result.Success, "应成功找到对角路径");
            Assert.AreEqual(start, result.Path[0]);
            Assert.AreEqual(goal, result.Path[^1]);

            int turns = 0;
            for (int i = 1; i < result.Path.Count - 1; i++)
            {
                int dx1 = result.Path[i].X - result.Path[i - 1].X;
                int dy1 = result.Path[i].Y - result.Path[i - 1].Y;
                int dx2 = result.Path[i + 1].X - result.Path[i].X;
                int dy2 = result.Path[i + 1].Y - result.Path[i].Y;
                if (dx1 != dx2 || dy1 != dy2)
                {
                    turns++;
                }
            }

            Assert.AreEqual(1, turns, "大跨度开阔对角路径转弯次数必须严格为 1 (标准 L 型，绝非多次折线的阶梯型)");
        }

        // 场景 18：验证 A* 与 Dijkstra 算法代价函数的明确区分
        [TestMethod]
        public void Scenario18_AStarAndDijkstraCostFunctionDistinction()
        {
            // 验证 A* 将转向纳入综合代价，而 Dijkstra 严格以网格跳数为步长代价。
            // map: 30x30 开阔地图; astar: A* 规划器; dijkstra: Dijkstra 规划器; start: 规划起点; goal: 对角目标点。
            var map = new GridMap(30, 30, 0.55f);
            var astar = new AStarPlanner();
            var dijkstra = new DijkstraPlanner();

            var start = new GridPoint(1, 1);
            var goal = new GridPoint(8, 8);

            var astarResult = astar.PlanPath(map, start, goal);
            var dijkstraResult = dijkstra.PlanPath(map, start, goal);

            Assert.IsTrue(astarResult.Success);
            Assert.IsTrue(dijkstraResult.Success);

            // Dijkstra 的代价是纯距离步数 (14步)，无转向惩罚
            Assert.AreEqual(14.0f, dijkstraResult.Cost, 1e-4f, "Dijkstra 代价函数应严格等于曼哈顿拓扑步数 14.0");

            // A* 规划出标准 L 型路径，代价为 14步 + 1次转向 (0.7) = 14.7
            Assert.AreEqual(14.7f, astarResult.Cost, 1e-4f, "A* 代价函数包含步长与转向惩罚");
        }

        // 场景 19：格间悬停维持双格锁与通道边防追尾
        [TestMethod]
        public void Scenario19_HoverBetweenGridsMaintainsDoubleLock()
        {
            // 验证机器人在两格之间行驶未扫到新地标时维持双格占用锁与通道边锁。
            // manager: 待测交通预约管理器; startGrid: 初始所处栅格; nextGrid: 前往目标栅格; followerGrid: 后方拟驶入机器人所在栅格。
            var manager = new GridReservationManager();
            var startGrid = new GridPoint(2, 2);
            var nextGrid = new GridPoint(3, 2);
            var followerGrid = new GridPoint(1, 2);

            manager.RegisterRobotInitialGrid(1, startGrid);
            manager.RegisterRobotInitialGrid(2, followerGrid);

            // 机器人 1 获得前往 nextGrid 的授权
            bool r1Authorized = manager.TryAcquireStepAuthorization(1, startGrid, nextGrid, out _);
            Assert.IsTrue(r1Authorized);

            // 机器人 1 处于两格中间，读码仍为 startGrid (未扫到 nextGrid)
            manager.UpdateOccupancyAndRelease(1, startGrid, nextGrid, startGrid);

            // 验证 1：原栅格与目标栅格均处于保护锁定状态
            Assert.IsTrue(manager.OccupiedGrids.ContainsKey(startGrid), "原栅格必须保持被机器人 1 占用");
            Assert.IsTrue(manager.ReservedGrids.ContainsKey(nextGrid), "目标栅格必须保持被机器人 1 预约");

            // 验证 2：后方机器人 2 尝试驶入原栅格 startGrid，必须被严正拒绝以防追尾
            bool r2Allowed = manager.TryAcquireStepAuthorization(2, followerGrid, startGrid, out string rejectReason);
            Assert.IsFalse(r2Allowed, "后方小车不得驶入前车尚未扫码交接的原栅格");
            Assert.IsTrue(rejectReason.Contains("占用"));
        }

        // 场景 20：扫码进入新地标中心原子交接与原资源释放
        [TestMethod]
        public void Scenario20_LandmarkConfirmationAtomicHandoverAndRelease()
        {
            // 验证机器人中心传感器对齐新地标后原子化确立新占用并彻底释放原栅格和边。
            var manager = new GridReservationManager();
            var startGrid = new GridPoint(4, 4);
            var nextGrid = new GridPoint(5, 4);

            manager.RegisterRobotInitialGrid(1, startGrid);
            manager.TryAcquireStepAuthorization(1, startGrid, nextGrid, out _);

            // 扫码确认：机器人 ConfirmedGrid 变为 nextGrid
            manager.UpdateOccupancyAndRelease(1, startGrid, nextGrid, nextGrid);

            Assert.IsTrue(manager.OccupiedGrids.ContainsKey(nextGrid), "新地标必须成为正式物理占用");
            Assert.IsFalse(manager.OccupiedGrids.ContainsKey(startGrid), "原栅格必须已被安全释放");
            Assert.IsFalse(manager.ReservedGrids.ContainsKey(nextGrid), "预约池中已无该目标");

            long edgeKey = GridReservationManager.GetUndirectedEdgeKey(startGrid, nextGrid);
            Assert.IsFalse(manager.ReservedEdges.ContainsKey(edgeKey), "通道无向边已被彻底释放");
        }

        // 场景 21：重新设置目标点后旧目标点与未走路径资源被彻底释放
        [TestMethod]
        public void Scenario21_RetargetReleasesOldTargetAndResidualReservations()
        {
            // 验证单机模式在重设导航目标时旧目标栅格与旧路径所有未通行预约资源被彻底清除释放。
            // app: 仿真外观门面实例; oldTarget: 原先设定的旧目标网格; newTarget: 重新设定的新目标网格。
            var app = new SimulationApp();
            var oldTarget = new GridPoint(14, 5);
            var newTarget = new GridPoint(14, 25);

            // 1. 下发旧导航目标 (14, 5)
            var res1 = app.NavigateRobot(1, oldTarget);
            Assert.IsTrue(res1.Success);
            Assert.IsTrue(app.Engine.ReservationManager.ReservedGrids.ContainsKey(oldTarget), "旧目标点必须已被预约");

            // 2. 在移动前或移动途中重新设定新目标 (14, 25)
            var res2 = app.NavigateRobot(1, newTarget);
            Assert.IsTrue(res2.Success);

            // 3. 验证旧目标点及其路径上的预约资源已被彻底移除
            Assert.IsFalse(app.Engine.ReservationManager.ReservedGrids.ContainsKey(oldTarget), "重设目标后旧目标点预约必须已被彻底释放");
            Assert.IsTrue(app.Engine.ReservationManager.ReservedGrids.ContainsKey(newTarget), "新目标点已被正确预约");
        }

        // 场景 22：长直道全速巡航停稳与终点严格正交航向对齐
        [TestMethod]
        public void Scenario22_StraightCruiseAndFinalOrthogonalHeadingAlignment()
        {
            // 验证机器人沿长直道连续平滑巡航到站，停稳后朝向精确对齐正交四方向之一。
            // robot: 目标机器人实体; mc: 运动控制器实例; headingDiff: 最终航向与正向角度偏差。
            var app = new SimulationApp();
            var target = new GridPoint(14, 8); // 向正上方延伸 6 个栅格的长直道

            var res = app.NavigateRobot(1, target);
            Assert.IsTrue(res.Success);

            // 仿真推进足够步数以平稳巡航到站
            app.RunHeadless(60);

            var robot = app.Engine.GetRobot(1);
            var mc = app.Engine.GetMotionController(1);

            Assert.IsTrue(mc.IsPathCompleted, "长直道巡航应平滑完成到达");
            Assert.AreEqual(target, robot.ConfirmedGrid, "机器人应精确停靠在目标栅格中央");
            Assert.IsTrue(robot.LinearVelocity <= robot.Config.SpeedTolerance, "停靠后线速度应完全降为 0");

            // 验证最终朝向：向上行驶对应 -PI/2 (-1.5708 rad)
            float expectedHeading = -(float)Math.PI * 0.5f;
            float headingDiff = Math.Abs(Pose2D.AngleDifference(expectedHeading, robot.Position.Heading));
            Assert.IsTrue(headingDiff < 0.05f, $"到达终点后朝向必须严格对齐正交正向 (向上 -90°)，当前偏角: {robot.Position.Heading * 180f / Math.PI:F1}°");
        }

        // 场景 23：转角栅格在机器人通过后被彻底安全释放 (杜绝橙色高亮残留)
        [TestMethod]
        public void Scenario23_CornerGridOccupancyIsSweptAfterPassing()
        {
            // 验证机器人走过 L 型折线路径后拐弯转角栅格的物理占用被彻底释放。
            // cornerTarget: 经过单次折线拐角的目标终点网格; cornerGrid: 路径中实际发生转向的拐角栅格。
            var app = new SimulationApp();
            // 起点 (14, 14)，拐角 (14, 10)，终点 (18, 10)
            var cornerTarget = new GridPoint(18, 10);
            var cornerGrid = new GridPoint(14, 10);

            var res = app.NavigateRobot(1, cornerTarget);
            Assert.IsTrue(res.Success);

            // 驱动仿真直到完全到达终点 (18, 10)
            app.RunHeadless(80);

            var robot = app.Engine.GetRobot(1);
            Assert.AreEqual(cornerTarget, robot.ConfirmedGrid, "机器人应到达最终目标");

            // 核心断言：拐角栅格 (14, 10) 绝不能残留在 OccupiedGrids 占用池中 (杜绝橙色高亮残留)
            Assert.IsFalse(app.Engine.ReservationManager.OccupiedGrids.ContainsKey(cornerGrid), "拐角栅格在机器人驶离后必须已被彻底释放，禁止残留橙色占用");

            // 并且只有终点网格保留占用
            Assert.IsTrue(app.Engine.ReservationManager.OccupiedGrids.ContainsKey(cornerTarget), "当前应仅由终点栅格确立占用");
        }

        // 场景 24：机器人行驶过程中严格贴合航道正中轴线且通过栅格物理中心
        [TestMethod]
        public void Scenario24_MovementStrictlyPassesThroughGridCentersAndMatchesTrack()
        {
            // 验证机器人在沿航段移动时横向偏离中心线误差严格受控于一毫米之内。
            // cellSize: 地图单网格物理尺寸; target: 导航目标网格; maxCrossTrackError: 行驶过程中的最大横向轨迹偏差。
            var app = new SimulationApp();
            float cellSize = app.CurrentMap.CellSizeMeters;
            var target = new GridPoint(14, 9); // 向正上方移动 5 个网格

            app.NavigateRobot(1, target);

            float maxCrossTrackError = 0f;
            float expectedCenterX = (14 + 0.5f) * cellSize; // 垂直航道 X 坐标应严格对齐第 14 列中心

            for (int i = 0; i < 40; i++)
            {
                app.Engine.Step();
                var r = app.Engine.GetRobot(1);
                if (r.LinearVelocity > 0.1f)
                {
                    float crossError = Math.Abs(r.Position.X - expectedCenterX);
                    if (crossError > maxCrossTrackError)
                    {
                        maxCrossTrackError = crossError;
                    }
                }
            }

            Assert.IsTrue(maxCrossTrackError < 1e-4f, $"小车在行驶过程中横向轨迹最大偏差为 {maxCrossTrackError:F5}m，必须严格对齐蓝色规划中心线");
        }

        // 场景 25：机器人原地转向遵循梯形角加减速平滑曲线且不发生角速度瞬态归零
        [TestMethod]
        public void Scenario25_RobotRotationFollowsTrapezoidalDecelerationProfile()
        {
            // 验证机器人在原地转向时角速度按角加速度平滑加速并按角减速度梯形减速到位。
            // robot: 测试机器人实体; dt: 仿真单步微元时间; maxObservedOmega: 转向过程中观测到的最大角速度峰值; lastOmegaBeforeStop: 停稳到位前最后一个步长记录的角速度; targetAngle: 原地目标旋转弧度; sawDeceleration: 减速过程观察标志。
            var robot = new SimulatedRobot(1, new GridPoint(5, 5), 0f, 0.55f);
            float targetAngle = (float)Math.PI * 0.5f; // 原地旋转 90 度
            robot.ExecuteCommand(RobotCommand.Rotate(targetAngle));

            float dt = 0.05f;
            float maxObservedOmega = 0f;
            float lastOmegaBeforeStop = 0f;
            bool sawDeceleration = false;

            for (int i = 0; i < 50 && robot.RemainingAngle > 0f; i++)
            {
                float prevOmega = Math.Abs(robot.AngularVelocity);
                robot.Step(dt, 0.55f);
                float curOmega = Math.Abs(robot.AngularVelocity);

                if (curOmega > maxObservedOmega)
                {
                    maxObservedOmega = curOmega;
                }

                if (prevOmega > 0.5f && curOmega < prevOmega && robot.RemainingAngle > 0f)
                {
                    sawDeceleration = true;
                }

                if (robot.RemainingAngle > 0f)
                {
                    lastOmegaBeforeStop = curOmega;
                }
            }

            Assert.IsTrue(maxObservedOmega > 0.5f, "转向过程中应加速至有效角速度");
            Assert.IsTrue(sawDeceleration, "转向到位前必须观测到按角减速度逐步减速的平滑制动过程");
            Assert.IsTrue(lastOmegaBeforeStop < maxObservedOmega, "到位前最后一个步长的角速度应显著低于峰值角速度，杜绝高速瞬间归零");
            Assert.AreEqual(0f, robot.RemainingAngle, 1e-4f, "目标旋转角度应精确完成");
        }

        // 场景 26：PID 闭环控制到站对准且无任何坐标瞬移
        [TestMethod]
        public void Scenario26_PidClosedLoopArrivalWithoutCoordinateTeleport()
        {
            // 验证机器人在运动控制器闭环 PID 驱动下平滑到站，全程单步位移严格受限于动力学速度且无坐标跳变。
            // app: 仿真外观门面实例; target: 导航目标网格; prevPos: 前一仿真步长的世界坐标; maxStepJump: 记录的最大单步位移跳变距离; r: 目标机器人实体; distToCenter: 最终距离栅格中心的实际物理残差。
            var app = new SimulationApp();
            var target = new GridPoint(14, 11); // 直行 3 格

            var navRes = app.NavigateRobot(1, target);
            Assert.IsTrue(navRes.Success);

            var r = app.Engine.GetRobot(1);
            var prevPos = r.Position;
            float maxStepJump = 0f;

            for (int i = 0; i < 50; i++)
            {
                app.Engine.Step();
                float stepDist = r.Position.DistanceTo(prevPos.X, prevPos.Y);
                if (stepDist > maxStepJump)
                {
                    maxStepJump = stepDist;
                }
                prevPos = r.Position;
            }

            var mc = app.Engine.GetMotionController(1);
            Assert.IsTrue(mc.IsPathCompleted, "小车应通过 PID 闭环平滑停稳到站");
            Assert.AreEqual(target, r.ConfirmedGrid, "地标中心应成功确认目标栅格");

            // 单步最大位移不应超过物理极值 (MaxSpeed * dt = 1.5 * 0.1 = 0.15m)，杜绝瞬移
            Assert.IsTrue(maxStepJump <= 0.16f, $"最大单步位移为 {maxStepJump:F3}m，严禁出现瞬移坐标跳变");

            // 最终物理中心偏差严格收敛在毫米级以内
            float targetPhysX = (target.X + 0.5f) * app.CurrentMap.CellSizeMeters;
            float targetPhysY = (target.Y + 0.5f) * app.CurrentMap.CellSizeMeters;
            float distToCenter = r.Position.DistanceTo(targetPhysX, targetPhysY);
            Assert.IsTrue(distToCenter <= 0.02f, $"最终停靠中心偏差为 {distToCenter:F4}m，应精准收敛在 2cm 以内");
        }

        // 场景 27：直段巡航至拐点减速平滑连续，杜绝中途误判急刹停顿与二次起步
        [TestMethod]
        public void Scenario27_StraightCruiseToCornerDeceleratesSmoothlyWithoutMidwayStopStall()
        {
            // 验证小车在直段高速巡航驶向转弯拐角时梯形连续减速，中途不发生速度断崖归零或状态提前变为停顿。
            // app: 仿真外观门面实例; cornerTarget: 经过单次折线拐角的目标终点网格; cornerGrid: 路径中实际发生转向的拐角栅格; r: 目标机器人实体; cornerPhysY: 拐角网格世界几何中心纵坐标; midwayStallDetected: 是否在中途检测到异常停顿标志。
            var app = new SimulationApp();
            var cornerTarget = new GridPoint(18, 10);
            var cornerGrid = new GridPoint(14, 10);

            var navRes = app.NavigateRobot(1, cornerTarget);
            Assert.IsTrue(navRes.Success);

            var r = app.Engine.GetRobot(1);
            float cellSize = app.CurrentMap.CellSizeMeters;
            float cornerPhysY = (cornerGrid.Y + 0.5f) * cellSize;
            bool midwayStallDetected = false;

            // 监测到达拐角之前的减速过程
            for (int i = 0; i < 40; i++)
            {
                app.Engine.Step();
                float distToCorner = Math.Abs(r.Position.Y - cornerPhysY);

                // 在距离拐角中心大于 0.05m 的中途路段
                if (distToCorner > 0.05f && distToCorner < 1.0f)
                {
                    // 若中途线速度小于 0.01m/s 或状态变为 Stopped，说明发生中途急刹顿挫
                    if (r.LinearVelocity < 0.01f && r.MotionState == RobotMotionState.Stopped)
                    {
                        midwayStallDetected = true;
                        break;
                    }
                }

                // 一旦拐弯完成开始横向行驶则退出前段监测
                if (r.Position.X > (14 + 0.6f) * cellSize)
                {
                    break;
                }
            }

            Assert.IsFalse(midwayStallDetected, "小车在驶向转弯拐角的过程中必须平滑减速，严禁在中途急刹停顿并二次起步");
        }

        // 场景 28：机器人在田字格十字交界非中心偏位状态下启动导航并安全归正到站
        [TestMethod]
        public void Scenario28_OffCenterRobotInIntersectionNavigatesSafelyToGoal()
        {
            // 验证机器人在四个栅格交界的十字路口偏位处启动导航，安全层加锁交叠栅格并平滑归正最终到达目标。
            // map: 仓储网格地图; engine: 仿真引擎实例; cellSize: 单网格物理边长; offCenterX: 田字格十字交汇世界横坐标; offCenterY: 田字格十字交汇世界纵坐标; robot: 初始置于十字交界处的仿真机器人实体; target: 导航目标网格; navRes: 导航调用返回结果; mc: 运动控制器实例; targetPhysX: 目标网格世界几何中心横坐标; targetPhysY: 目标网格世界几何中心纵坐标; distToGoal: 到达目标中心残差距离。
            var map = new GridMap(20, 20, 0.55f);
            var engine = new SimulationEngine(map, new SimulationConfig { FixedDeltaTime = 0.1f });
            float cellSize = 0.55f;

            // 将机器人精确放置在四个网格 (10,10), (11,10), (10,11), (11,11) 的十字交叉中心点
            float offCenterX = 11f * cellSize;
            float offCenterY = 11f * cellSize;
            var robot = new SimulatedRobot(1, offCenterX, offCenterY, 0f, cellSize);
            engine.AddRobot(robot);

            // 导航目标为右侧 (15, 10)
            var target = new GridPoint(15, 10);
            var navRes = engine.NavigateRobot(1, target);

            Assert.IsTrue(navRes.Success, "处于田字格交汇非中心偏位状态下的机器人应成功启动导航");

            // 验证交叠包络安全锁：田字格关键栅格已被安全纳入占用管理
            var manager = engine.ReservationManager;
            Assert.IsTrue(manager.OccupiedGrids.ContainsKey(new GridPoint(11, 10)) ||
                          manager.OccupiedGrids.ContainsKey(new GridPoint(10, 10)),
                          "安全层必须原子化锁定与车体交叠的田字格网格");

            var mc = engine.GetMotionController(1);

            // 仿真推进 100 步，驱动机器人就近归正并完成全程巡航
            for (int i = 0; i < 100; i++)
            {
                engine.Step();
            }

            Assert.IsTrue(mc.IsPathCompleted, "机器人应顺利完成就近归正与全程导航到达");
            Assert.AreEqual(target, robot.ConfirmedGrid, "地标感应器最终应精准对准目标栅格");

            float targetPhysX = (target.X + 0.5f) * cellSize;
            float targetPhysY = (target.Y + 0.5f) * cellSize;
            float distToGoal = robot.Position.DistanceTo(targetPhysX, targetPhysY);
            Assert.IsTrue(distToGoal <= 0.02f, $"最终停靠位置与目标中心偏差为 {distToGoal:F4}m，应小于 2cm");
        }

        // 场景 29：横向中心线循迹 PID 控制器在直角转弯后动态消除横向偏差并精准贴合航道
        [TestMethod]
        public void Scenario29_LateralCrossTrackPidDynamicallyEliminatesTrackingOffset()
        {
            // 验证机器人在完成拐弯转向切入新航道后横向循迹闭环控制器持续纠偏，确保行驶轨迹严格对准栅格中心线。
            // app: 仿真外观门面实例; cornerTarget: 包含直角拐弯的目标网格; navRes: 导航调用返回结果; r: 目标机器人实体; cellSize: 单网格物理边长; expectedY: 拐弯后水平航道标准中心纵坐标; maxHorizontalCrossTrackError: 水平巡航阶段记录的最大横向距离残差; mc: 运动控制器实例。
            var app = new SimulationApp();
            var cornerTarget = new GridPoint(18, 10);

            var navRes = app.NavigateRobot(1, cornerTarget);
            Assert.IsTrue(navRes.Success);

            var r = app.Engine.GetRobot(1);
            float cellSize = app.CurrentMap.CellSizeMeters;
            float expectedY = (10 + 0.5f) * cellSize; // 拐角后水平段 Y 应为 5.775m
            float maxHorizontalCrossTrackError = 0f;

            // 推进仿真直到到达终点
            for (int i = 0; i < 90; i++)
            {
                app.Engine.Step();

                // 当小车已拐过弯并进入水平直道巡航 (X 越过 15 列)
                if (r.Position.X > (15 + 0.5f) * cellSize && r.LinearVelocity > 0.1f)
                {
                    float lateralError = Math.Abs(r.Position.Y - expectedY);
                    if (lateralError > maxHorizontalCrossTrackError)
                    {
                        maxHorizontalCrossTrackError = lateralError;
                    }
                }
            }

            var mc = app.Engine.GetMotionController(1);
            Assert.IsTrue(mc.IsPathCompleted, "折线导航应顺利完成到达");
            Assert.AreEqual(cornerTarget, r.ConfirmedGrid, "最终地标应精准匹配目标点");

            // 验证在水平航道巡航期间的横向误差被 PID 牢牢控制在 1 毫米以内
            Assert.IsTrue(maxHorizontalCrossTrackError < 0.001f,
                $"水平航道巡航最大横向偏差为 {maxHorizontalCrossTrackError:F5}m，应通过横向 PID 严格收敛至 1mm 以内");
        }
    }
}
