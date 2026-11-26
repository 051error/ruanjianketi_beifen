using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Robots;

namespace WarehouseGridSimulation.Core.Motion
{
    /// <summary>
    /// 机器人运动控制器 (MotionController)。
    /// 职责：
    /// 1. 将离散路径节点列表与机器人当前物理位姿转换为机器人支持的基础指令；
    /// 2. 严格遵循控制闭环，禁止直接修改或瞬移机器人坐标；
    /// 3. 管理授权路段终点与停靠判定，包含位置容差与速度容差，有效防止单步超调 (Overshoot)。
    /// </summary>
    public class MotionController
    {
        public int RobotId { get; }
        public IReadOnlyList<GridPoint> CurrentPath { get; private set; }
        public int CurrentTargetIndex { get; private set; }
        public int AuthorizedIndex { get; private set; }
        public GridPoint LastConfirmedGrid { get; private set; }

        public bool HasPath => CurrentPath != null && CurrentPath.Count > 0;
        public bool IsPathCompleted { get; private set; } = true;
        public bool IsAwaitingAuthorization { get; private set; } = false;

        private readonly PidController _dockingPid = new PidController(2.0f, 0.05f, 0.1f, 0f, 0.2f);

        public GridPoint? CurrentTargetGrid => (CurrentPath != null && CurrentTargetIndex < CurrentPath.Count)
            ? CurrentPath[CurrentTargetIndex]
            : (GridPoint?)null;

        public MotionController(int robotId)
        {
            // 初始化机器人底层运动控制器。
            // robotId: 目标机器人编号。
            RobotId = robotId;
        }

        public void SetPath(IReadOnlyList<GridPoint> path, int initialAuthorizedIndex = 0, int startTargetIndex = -1)
        {
            // 为机器人配置新的规划路径、初始授权索引与起始目标节点。
            // path: 规划路径节点序列; initialAuthorizedIndex: 初始已获得通行许可的最大路径节点索引; startTargetIndex: 起始目标节点索引。
            CurrentPath = path ?? Array.Empty<GridPoint>();
            if (startTargetIndex >= 0 && startTargetIndex < CurrentPath.Count)
            {
                CurrentTargetIndex = startTargetIndex;
            }
            else
            {
                CurrentTargetIndex = (CurrentPath.Count > 1) ? 1 : 0;
            }
            LastConfirmedGrid = (CurrentPath.Count > 0) ? CurrentPath[0] : GridPoint.Empty;
            AuthorizedIndex = (initialAuthorizedIndex >= 0 && initialAuthorizedIndex < CurrentPath.Count)
                ? initialAuthorizedIndex
                : 0;
            IsPathCompleted = CurrentPath.Count == 0;
            IsAwaitingAuthorization = false;
            _dockingPid.Reset();
        }

        public void AdvanceConfirmedGrid(GridPoint newConfirmedGrid)
        {
            // 更新控制器内部记录的最新确认地标栅格。
            LastConfirmedGrid = newConfirmedGrid;
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
            _dockingPid.Reset();
        }

        private static float SnapToOrthogonalHeading(float heading)
        {
            // 将浮点航向角规约对齐至最接近的标准正交四方向之一。
            // heading: 原始浮点航向角弧度值。
            float norm = Pose2D.NormalizeAngle(heading);
            float halfPi = (float)Math.PI * 0.5f;
            float pi = (float)Math.PI;

            if (Math.Abs(norm - 0f) < 0.25f * pi) return 0f;
            if (Math.Abs(norm - halfPi) < 0.25f * pi) return halfPi;
            if (Math.Abs(norm - (-halfPi)) < 0.25f * pi) return -halfPi;
            return (norm > 0f) ? pi : -pi;
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

        private float CalculateRemainingStraightDistance(SimulatedRobot robot, float cellSizeMeters)
        {
            // 计算沿当前直线航段连续延伸到下一个转弯点或终点的累积直线距离。
            // robot: 控制目标机器人实体; cellSizeMeters: 单栅格物理尺寸。
            if (CurrentPath == null || CurrentTargetIndex >= CurrentPath.Count) return 0f;

            int endIdx = CurrentTargetIndex;
            while (endIdx < CurrentPath.Count - 1 && endIdx < AuthorizedIndex)
            {
                if (IsTurnAhead(CurrentPath, endIdx))
                {
                    break;
                }
                endIdx++;
            }

            var endGrid = CurrentPath[endIdx];
            float endPhysX = (endGrid.X + 0.5f) * cellSizeMeters;
            float endPhysY = (endGrid.Y + 0.5f) * cellSizeMeters;

            float dx = endPhysX - robot.Position.X;
            float dy = endPhysY - robot.Position.Y;
            return (float)Math.Sqrt(dx * dx + dy * dy);
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
            bool isHomeDocking = (CurrentTargetIndex == 0);
            bool isFlyByNode = !isFinalNode && !isAuthorizedSegmentEnd && !isNextTurn && !isHomeDocking;

            // 计算沿当前航段行进矢量的剩余投影距离，防止单步越界后 Atan2 反转误掉头
            GridPoint prevGrid = CurrentTargetIndex > 0 ? CurrentPath[CurrentTargetIndex - 1] : targetGrid;
            int segDx = targetGrid.X - prevGrid.X;
            int segDy = targetGrid.Y - prevGrid.Y;
            float segLen = (float)Math.Sqrt(segDx * segDx + segDy * segDy);
            float projRemaining = (segLen > 1e-4f) ? (dx * segDx + dy * segDy) / segLen : dist;

            // 2. 检查是否到达当前节点容差范围或穿过通过点
            if (isFinalNode)
            {
                // 到达最终目标点：当满足停靠精细距离容差且线速度降至容差内时宣告完成
                if (dist <= 0.01f || projRemaining <= 0.003f)
                {
                    if (robot.LinearVelocity <= robot.Config.SpeedTolerance)
                    {
                        IsPathCompleted = true;
                        _dockingPid.Reset();
                        return RobotCommand.Stop;
                    }
                    return RobotCommand.Stop;
                }
            }
            else if (isAuthorizedSegmentEnd || isNextTurn || isHomeDocking)
            {
                // 授权终点、前方拐弯或就近归正锚点：必须减速停稳在当前栅格中心
                if (dist <= 0.01f || projRemaining <= 0.003f)
                {
                    if (robot.LinearVelocity <= robot.Config.SpeedTolerance)
                    {
                        _dockingPid.Reset();
                        if (isAuthorizedSegmentEnd)
                        {
                            IsAwaitingAuthorization = true;
                            return RobotCommand.Stop;
                        }

                        // 转向点或归正锚点中心停稳后切换到下一节点进行转向，由真实运动学闭环推进
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
            }
            else
            {
                // 直道中间飞掠通过点 (Fly-by)：不减速连续巡航，一旦进入容差或越过中心立即切换到下一节点
                if (dist <= robot.Config.PositionTolerance || projRemaining <= 0.05f)
                {
                    CurrentTargetIndex++;
                    if (CurrentTargetIndex >= CurrentPath.Count)
                    {
                        IsPathCompleted = true;
                        return RobotCommand.Stop;
                    }
                }
            }

            // 重新计算向当前有效节点的矢量
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
            isHomeDocking = (CurrentTargetIndex == 0);
            isFlyByNode = !isFinalNode && !isAuthorizedSegmentEnd && !isNextTurn && !isHomeDocking;

            prevGrid = CurrentTargetIndex > 0 ? CurrentPath[CurrentTargetIndex - 1] : targetGrid;
            segDx = targetGrid.X - prevGrid.X;
            segDy = targetGrid.Y - prevGrid.Y;
            segLen = (float)Math.Sqrt(segDx * segDx + segDy * segDy);
            projRemaining = (segLen > 1e-4f) ? (dx * segDx + dy * segDy) / segLen : dist;

            // 3. 航向对齐判断：归正微运动段朝向锚点中心，常规航段使用严格正交航向锁定
            float desiredHeading = (segDx != 0 || segDy != 0)
                ? SnapToOrthogonalHeading((float)Math.Atan2(segDy, segDx))
                : (float)Math.Atan2(dy, dx);

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

            // 4. 到站或转弯前动力学减速与闭环 PID 精准对位判断
            bool requiresBrakingToStop = isFinalNode || isAuthorizedSegmentEnd || isNextTurn || isHomeDocking;

            if (requiresBrakingToStop)
            {
                // 仅当物理线速度已降为低速且剩余距离进入精对位蠕动门限时，才启动闭环微调
                if (projRemaining <= robot.Config.CreepThreshold && robot.LinearVelocity <= 0.25f)
                {
                    if (projRemaining <= 0.003f)
                    {
                        // 极度接近网格几何中心，彻底抱闸停稳
                        return RobotCommand.Stop;
                    }

                    // 闭环 PID 动态微调爬行速度并映射为单步平滑微调位移
                    _dockingPid.Kp = robot.Config.AlignmentKp;
                    _dockingPid.Kd = robot.Config.AlignmentKd;
                    _dockingPid.MaxOutput = robot.Config.CreepMaxSpeed;
                    _dockingPid.MinOutput = 0f;

                    float targetSpeed = _dockingPid.Compute(projRemaining, dt);
                    float stepMove = Math.Clamp(targetSpeed * dt, 0.001f, projRemaining);
                    return RobotCommand.Move(stepMove);
                }

                // 在高速减速阶段或距离尚远时，持续下发至目标中心的完整剩余投影距离，交由底层动力学梯形平滑减速，严禁单步位移截断
                return RobotCommand.Move(Math.Max(0.01f, projRemaining));
            }

            // 5. 直道连续全速巡航：下发延伸到拐弯点或终点的累积距离，避免小车每格误判减速
            float moveDistance = isFlyByNode
                ? Math.Max(dist, CalculateRemainingStraightDistance(robot, cellSizeMeters))
                : dist;

            return RobotCommand.Move(moveDistance);
        }
    }
}
