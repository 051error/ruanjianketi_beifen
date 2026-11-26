using System;
using System.Diagnostics;
using System.Threading;
using WarehouseGridSimulation.Core.App;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Planning;

namespace WarehouseGridSimulation.Cli.Runners
{
    /// <summary>
    /// 无 UI 单机导航仿真场景运行器。
    /// 负责单机交互点选目标、算法切换、动力学逐步推进与自动化批处理推演。
    /// </summary>
    public class SingleRobotRunner : ISimulationScenarioRunner
    {
        public string ScenarioName => "单机路径规划与动力学导航";
        public string Description => "测试单机在 30x30 正交地图中的 A*/Dijkstra 寻路、转弯成本优化及物理连续巡航到站。";

        public void RunInteractive(SimulationApp app)
        {
            // 启动交互式单机导航测试流程。
            // app: 仿真应用门面外观实例。
            if (app == null) throw new ArgumentNullException(nameof(app));

            Console.WriteLine();
            Console.WriteLine("==================================================");
            Console.WriteLine($"模式 1：{ScenarioName}");
            Console.WriteLine(Description);
            Console.WriteLine("==================================================");

            var robot = app.Engine.GetRobot(1);
            if (robot == null)
            {
                Console.WriteLine("[错误] 地图中未初始化编号为 1 的机器人！");
                return;
            }

            Console.WriteLine($"当前地图规格: {app.CurrentMap.Width}x{app.CurrentMap.Height} (单格 {app.CurrentMap.CellSizeMeters:F2}m)");
            Console.WriteLine($"机器人 #1 初始地标: {robot.ConfirmedGrid} | 物理坐标: ({robot.Position.X:F2}m, {robot.Position.Y:F2}m) | 朝向: {robot.Position.Heading * 180f / Math.PI:F1}°");
            Console.WriteLine();

            Console.Write("请选择寻路规划算法 [1: A* 算法 (默认，含转向成本) | 2: Dijkstra 算法 (纯拓扑距离)]: ");
            string algoInput = Console.ReadLine()?.Trim();
            IPathPlanner planner = (algoInput == "2") ? new DijkstraPlanner() : new AStarPlanner();
            string plannerName = (algoInput == "2") ? "Dijkstra 算法" : "A* 算法 (0.3 曼哈顿 + 0.7 转向成本)";

            Console.Write("请输入目标网格坐标 (格式: X Y，例如 18 10): ");
            string coordInput = Console.ReadLine()?.Trim();
            if (string.IsNullOrWhiteSpace(coordInput))
            {
                Console.WriteLine("[提示] 输入为空，已取消操作。");
                return;
            }

            string[] parts = coordInput.Split(new[] { ' ', ',', ';' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 2 || !int.TryParse(parts[0], out int targetX) || !int.TryParse(parts[1], out int targetY))
            {
                Console.WriteLine("[错误] 坐标格式无效，必须输入两个整数！");
                return;
            }

            var targetGrid = new GridPoint(targetX, targetY);
            Console.Write("请选择仿真推进方式 [1: 实时动画打印 (带延时观测) | 2: 高速批量运算]: ");
            string stepMode = Console.ReadLine()?.Trim();
            bool realTimeMode = (stepMode != "2");

            ExecuteNavigation(app, targetGrid, planner, plannerName, realTimeMode);
        }

        public void RunAutomated(SimulationApp app, string[] args)
        {
            // 通过命令行参数快速执行单机导航批处理。
            // app: 仿真门面实例; args: 外部传入的命令行参数列表。
            if (app == null) throw new ArgumentNullException(nameof(app));

            int targetX = 18;
            int targetY = 10;
            bool useDijkstra = false;
            bool realTime = false;

            if (args != null)
            {
                for (int i = 0; i < args.Length; i++)
                {
                    if (args[i] == "--x" && i + 1 < args.Length) int.TryParse(args[i + 1], out targetX);
                    if (args[i] == "--y" && i + 1 < args.Length) int.TryParse(args[i + 1], out targetY);
                    if (args[i] == "--algo" && i + 1 < args.Length && args[i + 1].Equals("dijkstra", StringComparison.OrdinalIgnoreCase)) useDijkstra = true;
                    if (args[i] == "--realtime") realTime = true;
                }
            }

            var planner = useDijkstra ? (IPathPlanner)new DijkstraPlanner() : new AStarPlanner();
            string plannerName = useDijkstra ? "Dijkstra" : "A*";
            ExecuteNavigation(app, new GridPoint(targetX, targetY), planner, plannerName, realTime);
        }

        private static void ExecuteNavigation(SimulationApp app, GridPoint target, IPathPlanner planner, string plannerName, bool realTime)
        {
            // 执行单机导航求解、路径输出与确定性步长推演。
            // target: 导航目标网格; planner: 选定规划器; plannerName: 算法显示名称; realTime: 是否以真实时钟间隔延时打印。
            var robot = app.Engine.GetRobot(1);
            var controller = app.Engine.GetMotionController(1);

            Console.WriteLine();
            Console.WriteLine($"[规划中] 正在使用 {plannerName} 从 {robot.ConfirmedGrid} 求解至目标 {target}...");

            var sw = Stopwatch.StartNew();
            var navResult = app.NavigateRobot(1, target, planner);
            sw.Stop();

            if (!navResult.Success)
            {
                Console.WriteLine($"[寻路失败] {navResult.Message}");
                return;
            }

            Console.WriteLine($"[规划成功] 耗时: {sw.ElapsedMilliseconds}ms | 路径节点数: {navResult.Path.Count} | 理论距离: {(navResult.Path.Count - 1) * app.CurrentMap.CellSizeMeters:F2}m");
            Console.Write("路径轨迹: ");
            for (int i = 0; i < navResult.Path.Count; i++)
            {
                Console.Write($"{navResult.Path[i]}");
                if (i < navResult.Path.Count - 1) Console.Write(" -> ");
            }
            Console.WriteLine();
            Console.WriteLine();

            Console.WriteLine("-------------------------------------------------------------------------------------------------");
            Console.WriteLine(" 时间(s) | 当前地标 |      连续物理坐标     | 线速度(m/s) | 角速度(°/s) |  朝向(°) | 控制器状态");
            Console.WriteLine("-------------------------------------------------------------------------------------------------");

            int maxSteps = 300;
            int step = 0;

            while (!controller.IsPathCompleted && step < maxSteps)
            {
                app.Step();
                step++;

                string stateDesc = controller.IsPathCompleted ? "已到达停稳" : (robot.LinearVelocity > 0.05f ? "连续巡航中" : (Math.Abs(robot.AngularVelocity) > 0.05f ? "原地自旋中" : "动力学启停"));

                Console.WriteLine(
                    $"{app.Engine.SimulationTime,7:F1} | " +
                    $"{robot.ConfirmedGrid,8} | " +
                    $"({robot.Position.X,5:F2}m, {robot.Position.Y,5:F2}m) | " +
                    $"{robot.LinearVelocity,11:F2} | " +
                    $"{robot.AngularVelocity * 180f / Math.PI,11:F1} | " +
                    $"{robot.Position.Heading * 180f / Math.PI,8:F1} | " +
                    $"{stateDesc}");

                if (realTime)
                {
                    Thread.Sleep(50); // 略微延时以便肉眼观测
                }
            }

            Console.WriteLine("-------------------------------------------------------------------------------------------------");
            if (controller.IsPathCompleted)
            {
                Console.WriteLine($"[导航顺利完成] 总用时: {app.Engine.SimulationTime:F1}s | 最终地标: {robot.ConfirmedGrid} | 终点速度: {robot.LinearVelocity:F3}m/s | 最终朝向: {robot.Position.Heading * 180f / Math.PI:F1}°");
            }
            else
            {
                Console.WriteLine($"[警告] 达到最大步数限制 ({maxSteps} 步)，未完全到达目标！");
            }
            Console.WriteLine();
        }
    }
}
