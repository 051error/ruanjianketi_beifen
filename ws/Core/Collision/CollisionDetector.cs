using System;
using System.Collections.Generic;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;
using WarehouseGridSimulation.Core.Robots;

namespace WarehouseGridSimulation.Core.Collision
{
    public enum CollisionType
    {
        RobotWithRobot = 0,
        RobotWithObstacle = 1
    }

    /// <summary>
    /// 仿真碰撞事件详情。
    /// 说明：本碰撞检测器属于离散动力学仿真的独立几何包络检测模块，
    /// 独立于调度预约机制，不可作为真实工业现场安全认证避障系统的等价替代。
    /// </summary>
    public class CollisionEvent
    {
        public float SimulationTime { get; }
        public CollisionType Type { get; }
        public int RobotId1 { get; }
        public int? RobotId2 { get; }
        public GridPoint? ObstacleGrid { get; }
        public Pose2D Location { get; }
        public string Description { get; }

        public CollisionEvent(
            float simTime,
            CollisionType type,
            int robotId1,
            int? robotId2,
            GridPoint? obstacleGrid,
            Pose2D location,
            string description)
        {
            SimulationTime = simTime;
            Type = type;
            RobotId1 = robotId1;
            RobotId2 = robotId2;
            ObstacleGrid = obstacleGrid;
            Location = location;
            Description = description;
        }

        public override string ToString() => $"[碰撞告警 @ {SimulationTime:F1}s] {Description}";
    }

    /// <summary>
    /// 独立碰撞检测器。
    /// 独立于交通预约层，基于物理几何尺寸直接评估机器人外包络碰撞风险。
    /// 模型说明：
    /// 1. 采用外包络圆半径近似车体外形 (半径 0.25m)；
    /// 2. 同时检测机器人静态位姿距离与跨越时间步的扫掠 (Swept) 位移，防止高速运动下的穿透 (Tunneling)；
    /// 3. 检测发生碰撞后触发事件，并向仿真引擎报告。
    /// </summary>
    public class CollisionDetector
    {
        public List<CollisionEvent> CheckCollisions(
            float simTime,
            IReadOnlyList<SimulatedRobot> robots,
            GridMap map)
        {
            var collisionList = new List<CollisionEvent>();
            if (robots == null || robots.Count == 0) return collisionList;

            float cellSize = map?.CellSizeMeters ?? 0.55f;

            // 1. 机器人与障碍物碰撞检测
            if (map != null)
            {
                foreach (var robot in robots)
                {
                    float r = robot.Config.CollisionRadius;
                    float rx = robot.Position.X;
                    float ry = robot.Position.Y;

                    // 快速筛选机器人周围相交的栅格
                    int minGx = Math.Max(0, (int)Math.Floor((rx - r) / cellSize));
                    int maxGx = Math.Min(map.Width - 1, (int)Math.Floor((rx + r) / cellSize));
                    int minGy = Math.Max(0, (int)Math.Floor((ry - r) / cellSize));
                    int maxGy = Math.Min(map.Height - 1, (int)Math.Floor((ry + r) / cellSize));

                    for (int gx = minGx; gx <= maxGx; gx++)
                    {
                        for (int gy = minGy; gy <= maxGy; gy++)
                        {
                            var gp = new GridPoint(gx, gy);
                            if (map.IsObstacle(gp))
                            {
                                // 圆形与 AABB 矩形碰撞检测
                                float left = gx * cellSize;
                                float right = (gx + 1) * cellSize;
                                float top = gy * cellSize;
                                float bottom = (gy + 1) * cellSize;

                                float closestX = Math.Clamp(rx, left, right);
                                float closestY = Math.Clamp(ry, top, bottom);

                                float dx = rx - closestX;
                                float dy = ry - closestY;
                                float distSq = dx * dx + dy * dy;

                                if (distSq < r * r)
                                {
                                    collisionList.Add(new CollisionEvent(
                                        simTime,
                                        CollisionType.RobotWithObstacle,
                                        robot.Id,
                                        null,
                                        gp,
                                        robot.Position,
                                        $"机器人 #{robot.Id} 与障碍物 {gp} 发生实体干涉碰撞 (距离 {Math.Sqrt(distSq):F3}m < 包络半径 {r:F3}m)"));
                                }
                            }
                        }
                    }
                }
            }

            // 2. 机器人与机器人碰撞检测 (静态包络圆重叠 + 扫掠检测)
            for (int i = 0; i < robots.Count; i++)
            {
                var r1 = robots[i];
                for (int j = i + 1; j < robots.Count; j++)
                {
                    var r2 = robots[j];

                    float combinedRadius = r1.Config.CollisionRadius + r2.Config.CollisionRadius;
                    float currentDist = r1.Position.DistanceTo(r2.Position);

                    if (currentDist < combinedRadius)
                    {
                        collisionList.Add(new CollisionEvent(
                            simTime,
                            CollisionType.RobotWithRobot,
                            r1.Id,
                            r2.Id,
                            null,
                            r1.Position,
                            $"机器人 #{r1.Id} 与 机器人 #{r2.Id} 发生车身碰撞 (中心距 {currentDist:F3}m < 安全阈值 {combinedRadius:F3}m)"));
                    }
                    else
                    {
                        // 扫掠碰撞检测：检查前一帧与当前帧两车线段间的最短距离，防止一步跨越穿越碰撞
                        float prevDist = r1.PreviousPosition.DistanceTo(r2.PreviousPosition);
                        // 当两车相对高速移动且中途可能交错
                        if (prevDist > combinedRadius && currentDist > combinedRadius)
                        {
                            // 计算两车在步长中点 (t = 0.5) 时的近似中心距离
                            float midX1 = (r1.PreviousPosition.X + r1.Position.X) * 0.5f;
                            float midY1 = (r1.PreviousPosition.Y + r1.Position.Y) * 0.5f;
                            float midX2 = (r2.PreviousPosition.X + r2.Position.X) * 0.5f;
                            float midY2 = (r2.PreviousPosition.Y + r2.Position.Y) * 0.5f;

                            float dxMid = midX1 - midX2;
                            float dyMid = midY1 - midY2;
                            float midDist = (float)Math.Sqrt(dxMid * dxMid + dyMid * dyMid);

                            if (midDist < combinedRadius)
                            {
                                collisionList.Add(new CollisionEvent(
                                    simTime,
                                    CollisionType.RobotWithRobot,
                                    r1.Id,
                                    r2.Id,
                                    null,
                                    r1.Position,
                                    $"机器人 #{r1.Id} 与 机器人 #{r2.Id} 在步长内发生扫掠交错碰撞 (中点距 {midDist:F3}m < 安全阈值 {combinedRadius:F3}m)"));
                            }
                        }
                    }
                }
            }

            return collisionList;
        }
    }
}
