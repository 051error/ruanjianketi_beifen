using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Cli.Runners;
using WarehouseGridSimulation.Core.App;

namespace WarehouseGridSimulation.Cli
{
    public class Program
    {
        public static void Main(string[] args)
        {
            // 仓储 RCS 无 UI 仿真控制台程序主入口。
            // args: 启动命令行参数数组。
            Console.OutputEncoding = System.Text.Encoding.UTF8;

            var runners = new Dictionary<string, ISimulationScenarioRunner>(StringComparer.OrdinalIgnoreCase)
            {
                ["1"] = new SingleRobotRunner(),
                ["single"] = new SingleRobotRunner()
            };

            // 如果携带命令行参数，进入非交互式快速批处理模式
            if (args != null && args.Length > 0)
            {
                var headlessApp = new SimulationApp();
                var runner = new SingleRobotRunner();
                runner.RunAutomated(headlessApp, args);
                return;
            }

            // 交互式控制台菜单
            while (true)
            {
                Console.WriteLine();
                Console.WriteLine("==================================================================");
                Console.WriteLine("        仓储多机器人 RCS 调度仿真系统 (无 UI 控制台推演终端)       ");
                Console.WriteLine("==================================================================");
                Console.WriteLine(" 1. 单机路径规划与动力学导航 (A* / Dijkstra / 转向惩罚 / 连续巡航)");
                Console.WriteLine(" 2. 多机协同调度与冲突消解推演 [预留扩展：WHCA* / CBS]");
                Console.WriteLine(" 3. 仓储上卸货分拣搬运任务全周期 [预留扩展：TaskManager 任务闭环]");
                Console.WriteLine(" 0. 退出系统");
                Console.WriteLine("==================================================================");
                Console.Write("请选择运行模式 [0-3]: ");

                string choice = Console.ReadLine()?.Trim();
                if (choice == "0" || string.Equals(choice, "exit", StringComparison.OrdinalIgnoreCase))
                {
                    Console.WriteLine("正在退出控制台终端，感谢使用。");
                    break;
                }

                if (choice == "1")
                {
                    var app = new SimulationApp();
                    runners["1"].RunInteractive(app);
                }
                else if (choice == "2")
                {
                    Console.WriteLine();
                    Console.WriteLine("[提示] 模式 2（多机协同调度推演）已在接口与调度层完全就绪，后续迭代直接注入 MultiRobotRunner 即可平滑接入！");
                }
                else if (choice == "3")
                {
                    Console.WriteLine();
                    Console.WriteLine("[提示] 模式 3（分拣搬运任务全周期）底层 TaskManager 状态机已就绪，后续迭代直接注入 SortingTaskRunner 即可平滑接入！");
                }
                else
                {
                    Console.WriteLine("[提示] 输入选项无效，请重新输入！");
                }
            }
        }
    }
}
