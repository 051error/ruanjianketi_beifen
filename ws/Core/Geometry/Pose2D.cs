using System;

namespace WarehouseGridSimulation.Core.Geometry
{
    /// <summary>
    /// 二维连续物理位姿 (X, Y 为以米为单位的世界坐标，Heading 为以弧度为单位的朝向角)。
    /// 朝向规定：0 弧度为 +X 方向 (向右)，PI/2 为 +Y 方向 (向下)，PI/-PI 为 -X 方向 (向左)，-PI/2 为 -Y 方向 (向上)。
    /// </summary>
    public readonly struct Pose2D : IEquatable<Pose2D>
    {
        public float X { get; }
        public float Y { get; }
        public float Heading { get; }

        public Pose2D(float x, float y, float heading)
        {
            X = x;
            Y = y;
            Heading = NormalizeAngle(heading);
        }

        public static float NormalizeAngle(float angle)
        {
            while (angle > (float)Math.PI) angle -= 2f * (float)Math.PI;
            while (angle <= -(float)Math.PI) angle += 2f * (float)Math.PI;
            return angle;
        }

        public static float AngleDifference(float targetAngle, float currentAngle)
        {
            return NormalizeAngle(targetAngle - currentAngle);
        }

        public static Pose2D Lerp(Pose2D from, Pose2D to, float alpha)
        {
            // 沿最短角度插值两个位姿。
            // from: 起始位姿; to: 目标位姿; alpha: 范围为零到一的插值系数。
            alpha = Math.Clamp(alpha, 0f, 1f);
            return new Pose2D(
                from.X + (to.X - from.X) * alpha,
                from.Y + (to.Y - from.Y) * alpha,
                from.Heading + AngleDifference(to.Heading, from.Heading) * alpha);
        }

        public float DistanceTo(Pose2D other)
        {
            float dx = X - other.X;
            float dy = Y - other.Y;
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        public float DistanceTo(float x, float y)
        {
            float dx = X - x;
            float dy = Y - y;
            return (float)Math.Sqrt(dx * dx + dy * dy);
        }

        public bool Equals(Pose2D other)
        {
            return Math.Abs(X - other.X) < 1e-4f &&
                   Math.Abs(Y - other.Y) < 1e-4f &&
                   Math.Abs(Heading - other.Heading) < 1e-4f;
        }

        public override bool Equals(object obj) => obj is Pose2D other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(X, Y, Heading);

        public override string ToString() => $"({X:F2}m, {Y:F2}m, {Heading * 180f / Math.PI:F1}°)";
    }
}
