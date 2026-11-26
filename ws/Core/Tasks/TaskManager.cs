using System;
using System.Collections.Generic;
using System.Linq;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;
using WarehouseGridSimulation.Core.Planning;
using WarehouseGridSimulation.Core.Robots;

namespace WarehouseGridSimulation.Core.Tasks
{
    /// <summary>
    /// 任务管理器 (TaskManager)。
    /// 负责：
    /// 1. 任务创建与队列管理；
    /// 2. 最近空闲机器人智能分配；
    /// 3. 可达性校验与不可达失败处理；
    /// 4. 模拟上货/卸货等待时长，严禁到达后直接瞬间完成；
    /// 5. 任务全生命周期推进。
    /// </summary>
    public class TaskManager
    {
        private readonly List<WarehouseTask> _tasks = new List<WarehouseTask>();
        private int _nextTaskId = 1;

        public IReadOnlyList<WarehouseTask> Tasks => _tasks;

        public event Action<WarehouseTask> TaskStatusChanged;
        public event Action<WarehouseTask, string> TaskFailed;

        public void Clear()
        {
            _tasks.Clear();
            _nextTaskId = 1;
        }

        public WarehouseTask CreateTask(Station pickStation, Station dropStation, int priority = 1)
        {
            if (pickStation == null) throw new ArgumentNullException(nameof(pickStation));
            if (dropStation == null) throw new ArgumentNullException(nameof(dropStation));

            var task = new WarehouseTask(_nextTaskId++, pickStation, dropStation, priority);
            _tasks.Add(task);
            TaskStatusChanged?.Invoke(task);
            return task;
        }

        /// <summary>
        /// 轮询并分配待处理任务至最近的空闲机器人。
        /// </summary>
        public void AssignPendingTasks(
            IReadOnlyList<SimulatedRobot> robots,
            GridMap map,
            IPathPlanner planner,
            Action<SimulatedRobot, WarehouseTask, IReadOnlyList<GridPoint>> onTaskAssigned)
        {
            var pendingTasks = _tasks.Where(t => t.Status == TaskStatus.Pending).OrderByDescending(t => t.Priority).ToList();
            if (pendingTasks.Count == 0) return;

            foreach (var task in pendingTasks)
            {
                // 寻找没有分配任务且没有载货的空闲机器人
                var availableRobots = robots.Where(r => r.CurrentTaskId == null && !r.HasCargo).ToList();
                if (availableRobots.Count == 0) break;

                // 按曼哈顿距离选取最近的机器人
                SimulatedRobot bestRobot = null;
                int bestDist = int.MaxValue;
                IReadOnlyList<GridPoint> bestPath = null;

                foreach (var robot in availableRobots)
                {
                    int dist = robot.GridPosition.ManhattanDistance(task.PickStation.Position);
                    if (dist < bestDist)
                    {
                        // 校验可达性 (自动绕开其他停泊闲置的机器人)
                        var planResult = planner.PlanPath(map, robot.GridPosition, task.PickStation.Position, pt =>
                            pt != task.PickStation.Position && robots.Any(rb => rb.Id != robot.Id && rb.GridPosition == pt && rb.CurrentTaskId == null));
                        if (planResult.Success)
                        {
                            bestDist = dist;
                            bestRobot = robot;
                            bestPath = planResult.Path;
                        }
                    }
                }

                if (bestRobot != null && bestPath != null)
                {
                    task.AssignedRobotId = bestRobot.Id;
                    task.Status = TaskStatus.MovingToPick;
                    bestRobot.CurrentTaskId = task.Id;

                    TaskStatusChanged?.Invoke(task);
                    onTaskAssigned?.Invoke(bestRobot, task, bestPath);
                }
                else
                {
                    // 无法从任何可用车规划到上货点，若所有车都不可达，检查上货点本身是否连通
                    bool startBlocked = map.IsObstacle(task.PickStation.Position);
                    if (startBlocked)
                    {
                        task.Status = TaskStatus.Failed;
                        task.FailureReason = $"上货点 {task.PickStation.Name} 处于不可通行区域";
                        TaskFailed?.Invoke(task, task.FailureReason);
                        TaskStatusChanged?.Invoke(task);
                    }
                }
            }
        }

        /// <summary>
        /// 单步更新进行中任务的等待时间与状态转换。
        /// </summary>
        public void UpdateActiveTasks(
            float dt,
            IReadOnlyList<SimulatedRobot> robots,
            GridMap map,
            IPathPlanner planner,
            Action<SimulatedRobot, WarehouseTask, IReadOnlyList<GridPoint>> onProceedToDrop,
            Action<SimulatedRobot, WarehouseTask> onTaskCompleted)
        {
            var activeTasks = _tasks.Where(t => t.Status != TaskStatus.Pending &&
                                                t.Status != TaskStatus.Completed &&
                                                t.Status != TaskStatus.Failed).ToList();

            foreach (var task in activeTasks)
            {
                var robot = robots.FirstOrDefault(r => r.Id == task.AssignedRobotId);
                if (robot == null) continue;

                switch (task.Status)
                {
                    case TaskStatus.WaitingPick:
                        // 等待模拟上货时长
                        task.WaitTimer += dt;
                        if (task.WaitTimer >= task.LoadingTimeSeconds)
                        {
                            // 模拟上货完成，装载货物
                            robot.HasCargo = true;
                            task.WaitTimer = 0f;

                            // 规划前往卸货点的路径 (自动绕开其他停泊闲置的机器人)
                            var planResult = planner.PlanPath(map, robot.GridPosition, task.DropStation.Position, pt =>
                                pt != task.DropStation.Position && robots.Any(rb => rb.Id != robot.Id && rb.GridPosition == pt && rb.CurrentTaskId == null));
                            if (planResult.Success)
                            {
                                task.Status = TaskStatus.MovingToDrop;
                                TaskStatusChanged?.Invoke(task);
                                onProceedToDrop?.Invoke(robot, task, planResult.Path);
                            }
                            else
                            {
                                // 卸货点不可达，标记任务失败
                                task.Status = TaskStatus.Failed;
                                task.FailureReason = $"无法到达卸货点 {task.DropStation.Name}: {planResult.ErrorMessage}";
                                robot.CurrentTaskId = null;
                                TaskFailed?.Invoke(task, task.FailureReason);
                                TaskStatusChanged?.Invoke(task);
                            }
                        }
                        break;

                    case TaskStatus.WaitingDrop:
                        // 等待模拟卸货时长
                        task.WaitTimer += dt;
                        if (task.WaitTimer >= task.UnloadingTimeSeconds)
                        {
                            // 模拟卸货完成，卸下货物
                            robot.HasCargo = false;
                            task.Status = TaskStatus.Completed;
                            robot.CurrentTaskId = null;
                            task.WaitTimer = 0f;

                            TaskStatusChanged?.Invoke(task);
                            onTaskCompleted?.Invoke(robot, task);
                        }
                        break;
                }
            }
        }
    }
}
