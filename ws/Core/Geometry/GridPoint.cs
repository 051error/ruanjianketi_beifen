using System;

namespace WarehouseGridSimulation.Core.Geometry
{
    /// <summary>
    /// 二维栅格离散坐标 (X 对应列，从 0 到 Width - 1；Y 对应行，从 0 到 Height - 1)。
    /// </summary>
    public readonly struct GridPoint : IEquatable<GridPoint>
    {
        public int X { get; }
        public int Y { get; }

        public GridPoint(int x, int y)
        {
            X = x;
            Y = y;
        }

        public static GridPoint Empty => new GridPoint(0, 0);

        public bool Equals(GridPoint other) => X == other.X && Y == other.Y;

        public override bool Equals(object obj) => obj is GridPoint other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(X, Y);

        public static bool operator ==(GridPoint left, GridPoint right) => left.Equals(right);

        public static bool operator !=(GridPoint left, GridPoint right) => !left.Equals(right);

        public static GridPoint operator +(GridPoint a, GridPoint b) => new GridPoint(a.X + b.X, a.Y + b.Y);

        public static GridPoint operator -(GridPoint a, GridPoint b) => new GridPoint(a.X - b.X, a.Y - b.Y);

        /// <summary>
        /// 曼哈顿距离。
        /// </summary>
        public int ManhattanDistance(GridPoint other)
        {
            return Math.Abs(X - other.X) + Math.Abs(Y - other.Y);
        }

        /// <summary>
        /// 欧式距离。
        /// </summary>
        public double EuclideanDistance(GridPoint other)
        {
            int dx = X - other.X;
            int dy = Y - other.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        public override string ToString() => $"({X}, {Y})";
    }
}
