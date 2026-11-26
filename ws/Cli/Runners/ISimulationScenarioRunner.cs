using System;
using WarehouseGridSimulation.Core.App;

namespace WarehouseGridSimulation.Cli.Runners
{
    /// <summary>
    /// 无 UI 仿真场景运行器通用契约接口。
    /// 统一兼容单机导航、多机协同调度推演与仓储分拣搬运任务全闭环。
    /// </summary>
    public interface ISimulationScenarioRunner
    {
        string ScenarioName { get; }
        string Description { get; }

        void RunInteractive(SimulationApp app);

        void RunAutomated(SimulationApp app, string[] args);
    }
}
