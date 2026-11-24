using WarehouseGridSimulation.Core.Map;

namespace WarehouseGridSimulation.Core.Tasks
{
    /// <summary>
    /// 仓储搬运任务状态。
    /// </summary>
    public enum TaskStatus
    {
        Pending = 0,         // 待分配
        MovingToPick = 1,    // 前往上货点
        WaitingPick = 2,     // 等待上货
        MovingToDrop = 3,    // 前往卸货点
        WaitingDrop = 4,     // 等待卸货
        Completed = 5,       // 已完成
        Failed = 6           // 失败
    }

    /// <summary>
    /// 仓储搬运任务 (上货点 -> 卸货点)。
    /// </summary>
    public class WarehouseTask
    {
        public int Id { get; }
        public Station PickStation { get; }
        public Station DropStation { get; }
        public int Priority { get; set; }

        public int? AssignedRobotId { get; set; }
        public TaskStatus Status { get; set; } = TaskStatus.Pending;
        public string FailureReason { get; set; }

        public float WaitTimer { get; set; } = 0f;
        public float LoadingTimeSeconds { get; set; } = 1.0f;
        public float UnloadingTimeSeconds { get; set; } = 1.0f;

        public WarehouseTask(int id, Station pickStation, Station dropStation, int priority = 1)
        {
            Id = id;
            PickStation = pickStation;
            DropStation = dropStation;
            Priority = priority;
        }

        public override string ToString() => $"[Task #{Id} P{Priority}: {PickStation?.Name}->{DropStation?.Name} ({Status}) Robot={AssignedRobotId}]";
    }
}
