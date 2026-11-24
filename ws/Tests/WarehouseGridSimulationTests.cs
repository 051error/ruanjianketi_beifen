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
    }
}
