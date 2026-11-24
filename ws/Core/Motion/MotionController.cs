using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Robots;

namespace WarehouseGridSimulation.Core.Motion
{
    /// <summary>
    /// 机器人运动控制器 (MotionController)。
    /// 职责：
    /// 1. 将离散路径节点列表与机器人当前物理位姿转换为机器人支持的纯基础指令 (MoveForward, TurnLeft, TurnRight, Stop)；
    /// 2. 严格遵循控制闭环，禁止直接修改或瞬移机器人坐标；
    /// 3. 管理授权路段终点与停靠判定，包含位置容差与速度容差，有效防止单步超调 (Overshoot)。
    /// </summary>
    public class MotionController
    {
        public int RobotId { get; }
        public IReadOnlyList<GridPoint> CurrentPath { get; private set; }
        public int CurrentTargetIndex { get; private set; }
        public int AuthorizedIndex { get; private set; }

        public bool HasPath => CurrentPath != null && CurrentPath.Count > 0;
        public bool IsPathCompleted { get; private set; } = true;
        public bool IsAwaitingAuthorization { get; private set; } = false;

        public GridPoint? CurrentTargetGrid => (CurrentPath != null && CurrentTargetIndex < CurrentPath.Count)
            ? CurrentPath[CurrentTargetIndex]
            : (GridPoint?)null;

        public MotionController(int robotId)
        {
            // 初始化机器人底层运动控制器。
            // robotId: 目标机器人编号。
            RobotId = robotId;
        }

        public void SetPath(IReadOnlyList<GridPoint> path, int initialAuthorizedIndex = -1)
        {
            // 为机器人配置新的规划路径与初始授权索引。
            // path: 规划路径节点序列; initialAuthorizedIndex: 初始已获得通行许可的最大路径节点索引。
            CurrentPath = path ?? Array.Empty<GridPoint>();
            CurrentTargetIndex = (CurrentPath.Count > 1) ? 1 : 0;
            AuthorizedIndex = initialAuthorizedIndex >= 0 ? initialAuthorizedIndex : CurrentPath.Count - 1;
            IsPathCompleted = CurrentPath.Count == 0;
            IsAwaitingAuthorization = false;
        }

        public void UpdateAuthorization(int authorizedIndex)
        {
            // 推进更新当前已获得通行授权的最大路径节点索引。
            // authorizedIndex: 最新获准通行的路径节点索引。
            AuthorizedIndex = Math.Min(authorizedIndex, CurrentPath != null ? CurrentPath.Count - 1 : 0);
            IsAwaitingAuthorization = false;
        }

        public void ClearPath()
        {
            // 清空当前控制路径并重置控制状态。
            CurrentPath = Array.Empty<GridPoint>();
            CurrentTargetIndex = 0;
            AuthorizedIndex = 0;
            IsPathCompleted = true;
            IsAwaitingAuthorization = false;
        }

        private static bool IsTurnAhead(IReadOnlyList<GridPoint> path, int index)
        {
            // 判定指定路径节点处是否需要改变行进方向。
            // index: 当前节点在路径中的索引。
            if (index <= 0 || index >= path.Count - 1) return false;
            int dx1 = path[index].X - path[index - 1].X;
            int dy1 = path[index].Y - path[index - 1].Y;
            int dx2 = path[index + 1].X - path[index].X;
            int dy2 = path[index + 1].Y - path[index].Y;
            return dx1 != dx2 || dy1 != dy2;
        }

        public RobotCommand ComputeNextCommand(SimulatedRobot robot, float cellSizeMeters, float dt)
        {
            // 计算当前仿真步长下应下发给机器人的基础控制指令。
            // robot: 控制目标机器人体。
            if (robot == null || !HasPath || IsPathCompleted)
            {
                return RobotCommand.Stop;
            }

            // 1. 检查授权是否限制前进
            if (CurrentTargetIndex > AuthorizedIndex)
            {
                // 当前目标尚未获得交通预约授权，机器人必须减速并在上一网格中心平稳停靠等待
                IsAwaitingAuthorization = true;
                return RobotCommand.Stop;
            }

            IsAwaitingAuthorization = false;
            GridPoint targetGrid = CurrentPath[CurrentTargetIndex];
            float targetPhysX = (targetGrid.X + 0.5f) * cellSizeMeters;
            float targetPhysY = (targetGrid.Y + 0.5f) * cellSizeMeters;

            float dx = targetPhysX - robot.Position.X;
            float dy = targetPhysY - robot.Position.Y;
            float dist = (float)Math.Sqrt(dx * dx + dy * dy);

            bool isFinalNode = (CurrentTargetIndex == CurrentPath.Count - 1);
            bool isAuthorizedSegmentEnd = (CurrentTargetIndex == AuthorizedIndex);
            bool isNextTurn = (CurrentTargetIndex > 0 && CurrentTargetIndex < CurrentPath.Count - 1) &&
                              IsTurnAhead(CurrentPath, CurrentTargetIndex);

            // 2. 检查是否到达当前节点容差范围
            if (dist <= robot.Config.PositionTolerance)
            {
                if (isFinalNode)
                {
                    // 到达最终目标点，要求速度降至静止容差
                    if (robot.LinearVelocity <= robot.Config.SpeedTolerance)
                    {
                        IsPathCompleted = true;
                        return RobotCommand.Stop;
                    }
                    return RobotCommand.Stop;
                }

                if (isAuthorizedSegmentEnd || isNextTurn)
                {
                    // 授权终点或前方拐弯：必须减速停稳在当前栅格中心
                    if (robot.LinearVelocity <= robot.Config.SpeedTolerance)
                    {
                        if (isAuthorizedSegmentEnd)
                        {
                            IsAwaitingAuthorization = true;
                            return RobotCommand.Stop;
                        }

                        // 转向点中心停稳后切换到下一节点进行转向
                        CurrentTargetIndex++;
                        if (CurrentTargetIndex >= CurrentPath.Count)
                        {
                            IsPathCompleted = true;
                            return RobotCommand.Stop;
                        }
                    }
                    else
                    {
                        return RobotCommand.Stop;
                    }
                }
                else
                {
                    // 直道中间节点 (Fly-by)：不减速连续巡航，直接切换到下一个目标节点
                    CurrentTargetIndex++;
                    if (CurrentTargetIndex >= CurrentPath.Count)
                    {
                        IsPathCompleted = true;
                        return RobotCommand.Stop;
                    }
                }

                // 重新计算向新节点的矢量
                targetGrid = CurrentPath[CurrentTargetIndex];
                targetPhysX = (targetGrid.X + 0.5f) * cellSizeMeters;
                targetPhysY = (targetGrid.Y + 0.5f) * cellSizeMeters;
                dx = targetPhysX - robot.Position.X;
                dy = targetPhysY - robot.Position.Y;
                dist = (float)Math.Sqrt(dx * dx + dy * dy);
                isFinalNode = (CurrentTargetIndex == CurrentPath.Count - 1);
                isAuthorizedSegmentEnd = (CurrentTargetIndex == AuthorizedIndex);
                isNextTurn = (CurrentTargetIndex > 0 && CurrentTargetIndex < CurrentPath.Count - 1) &&
                             IsTurnAhead(CurrentPath, CurrentTargetIndex);
            }

            // 3. 航向对齐判断 (方法 1：直行与转向严格互斥，必须先停稳才允许转向)
            float desiredHeading = (float)Math.Atan2(dy, dx);
            float angleDiff = Pose2D.AngleDifference(desiredHeading, robot.Position.Heading);

            if (Math.Abs(angleDiff) > robot.Config.HeadingTolerance)
            {
                // 若带有线速度，先彻底制动停稳在中心，再执行原地自旋
                if (robot.LinearVelocity > robot.Config.SpeedTolerance)
                {
                    return RobotCommand.Stop;
                }

                // 下发转向 (角度) 信号
                return RobotCommand.Rotate(angleDiff);
            }

            // 4. 航向已对齐，准备直线前进或到站前动力学减速
            float stoppingDistance = (robot.LinearVelocity * robot.LinearVelocity) / (2f * robot.Config.LinearDeceleration);
            bool requiresBrakingToStop = isFinalNode || isAuthorizedSegmentEnd || isNextTurn;

            if (requiresBrakingToStop && dist <= stoppingDistance + robot.Config.PositionTolerance)
            {
                // 距离停靠点小于制动距离：发送 Stop 指令自然减速滑行至目标点中心
                return RobotCommand.Stop;
            }

            // 正常向前连续平稳行驶：下发前进 (位移) 信号，携带当前段目标距离
            return RobotCommand.Move(dist);
        }
    }
}
