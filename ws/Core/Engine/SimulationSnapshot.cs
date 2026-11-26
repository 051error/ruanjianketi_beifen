using System.Collections.Generic;
using WarehouseGridSimulation.Core.Collision;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Robots;
using WarehouseGridSimulation.Core.Tasks;

namespace WarehouseGridSimulation.Core.Engine
{
    public class RobotSnapshot
    {
        public int Id { get; init; }
        public Pose2D Position { get; init; }
        public Pose2D PreviousPosition { get; init; }
        public GridPoint GridPosition { get; init; }
        public float LinearVelocity { get; init; }
        public float AngularVelocity { get; init; }
        public RobotMotionState MotionState { get; init; }
        public bool HasCargo { get; init; }
        public int? CurrentTaskId { get; init; }
        public IReadOnlyList<GridPoint> CurrentPath { get; init; }
        public int CurrentTargetIndex { get; init; }
        public GridPoint? TargetGrid { get; init; }
        public bool IsAwaitingAuthorization { get; init; }
        public float WaitTimeSeconds { get; init; }
    }

    public class TaskSnapshot
    {
        public int Id { get; init; }
        public string PickName { get; init; }
        public GridPoint PickPosition { get; init; }
        public string DropName { get; init; }
        public GridPoint DropPosition { get; init; }
        public int Priority { get; init; }
        public TaskStatus Status { get; init; }
        public int? AssignedRobotId { get; init; }
        public float WaitTimer { get; init; }
        public string FailureReason { get; init; }
    }

    /// <summary>
    /// 每一仿真步发布的不可变权威状态快照，供 UI 渲染和单元测试读取。
    /// 一旦发布，仿真引擎不会修改已发布的对象实例，保证完全的线程安全。
    /// </summary>
    public class SimulationSnapshot
    {
        public long Tick { get; init; }
        public float SimulationTime { get; init; }
        public bool IsRunning { get; init; }
        public bool IsPaused { get; init; }

        public IReadOnlyList<RobotSnapshot> Robots { get; init; }
        public IReadOnlyList<TaskSnapshot> Tasks { get; init; }
        public IReadOnlyDictionary<GridPoint, int> OccupiedGrids { get; init; }
        public IReadOnlyDictionary<GridPoint, int> ReservedGrids { get; init; }
        public IReadOnlyList<CollisionEvent> CollisionEvents { get; init; }
        public IReadOnlyList<string> LogEvents { get; init; }
    }
}
