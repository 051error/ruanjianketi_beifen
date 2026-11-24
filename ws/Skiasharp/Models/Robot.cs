using System;
using System.Collections.Generic;
using System.Drawing;
using SkiaSharp;
using WarehouseGridSimulation.Skia.Algorithms;

namespace WarehouseGridSimulation.Skia.Models
{
    /// <summary>
    /// 机器人正交朝向枚举。
    /// </summary>
    public enum RobotOrientation
    {
        Up = 0,
        Right = 1,
        Down = 2,
        Left = 3
    }

    /// <summary>
    /// 机器人高层状态机状态。
    /// </summary>
    public enum RobotNavState
    {
        /// <summary>
        /// 闲置待命状态（允许修改算法与设定参数）。
        /// </summary>
        Idle = 0,

        /// <summary>
        /// 手动移动控制状态（通过键盘或界面按钮进行手动前进或转向）。
        /// </summary>
        ManualMoving = 1,

        /// <summary>
        /// 自动移动导航状态（基于寻路算法规划路径自主导航）。
        /// </summary>
        AutoMoving = 2,

        /// <summary>
        /// 减速制动状态（正在平滑制动至安全栅格）。
        /// </summary>
        Braking = 3
    }

    /// <summary>
    /// 机器人底层单步物理动作状态。
    /// </summary>
    public enum RobotActionState
    {
        Idle = 0,
        Moving = 1,
        Turning = 2
    }

    /// <summary>
    /// 表示离散仓储栅格中运动的单个机器人（AGV），内置自动寻路状态机与连续直线平滑加减速动力学。
    /// </summary>
    public class Robot
    {
        public const float ChassisLengthMeters = 0.42f;
        public const float ChassisWidthMeters = 0.38f;

        public int GridX { get; set; }
        public int GridY { get; set; }
        public int TargetGridX { get; set; }
        public int TargetGridY { get; set; }
        public RobotOrientation Orientation { get; set; }

        public RobotNavState NavState { get; private set; }
        public RobotActionState ActionState { get; private set; }
        public RobotActionState State => ActionState;

        public PathAlgorithm CurrentAlgorithm { get; private set; }
        public Point? NavigationTarget { get; private set; }
        public List<Point> CurrentPath { get; private set; }

        public event Action<RobotNavState> NavStateChanged;
        public event Action TargetReached;

        public float PositionX { get; private set; }
        public float PositionY { get; private set; }
        public float CurrentAngle { get; private set; }

        public float MaxSpeed { get; set; }
        public float Acceleration { get; set; }
        public float CurrentSpeed { get; private set; }

        public float MaxAngularSpeed { get; set; }
        public float AngularAcceleration { get; set; }
        public float CurrentAngularVelocity { get; private set; }

        /// <summary>
        /// 外部调度接管开关。为 true 时机器人跑完一段后不再自行续接下一段，
        /// 必须停在栅格中心等待调度器授予受控区域通行许可，杜绝绕过许可闸门。
        /// </summary>
        public bool ExternalScheduling { get; set; }

        /// <summary>
        /// 是否已停在栅格中心等待调度器授予下一段通行许可。
        /// </summary>
        public bool IsAwaitingPermission { get; private set; }

        private readonly Queue<Point> _pathQueue = new Queue<Point>();

        private Point _segmentStartGrid;
        private int _segmentTotalSteps;
        private int _segmentStepsDequeued;

        // 调度器本次授权的连续直行格数上限（0 表示不限，用于单机模式）
        private int _grantedStepLimit;

        private bool _isBrakingToStop;
        private float _brakeInitialSpeed;
        private float _brakeTotalTime;
        private float _brakeDistance;

        private float _startPhysX;
        private float _startPhysY;
        private float _targetPhysX;
        private float _targetPhysY;

        private float _startAngle;
        private float _targetAngle;
        private float _turnElapsed;
        private float _turnDuration;
        private float _turnTotalAngle;
        private float _turnAccelTime;
        private float _turnConstTime;
        private float _turnPeakSpeed;
        private float _turnDirectionSign;

        private float _moveElapsed;
        private float _totalMoveTime;
        private float _accelTime;
        private float _constTime;
        private float _peakSpeed;
        private float _moveDistanceMeters;

        public Robot(int gridX, int gridY, float cellSizeMeters, RobotOrientation orientation)
        {
            // 初始化机器人实例、状态机与基础物理位姿。
            // gridX: 初始网格列索引。
            // gridY: 初始网格行索引。
            // cellSizeMeters: 单个网格物理边长（米）。
            // orientation: 初始正交朝向。
            GridX = gridX;
            GridY = gridY;
            TargetGridX = gridX;
            TargetGridY = gridY;
            Orientation = orientation;

            NavState = RobotNavState.Idle;
            ActionState = RobotActionState.Idle;
            CurrentAlgorithm = PathAlgorithm.AStar;
            CurrentPath = new List<Point>();

            MaxSpeed = 1.5f;
            Acceleration = 2.0f;
            CurrentSpeed = 0f;

            MaxAngularSpeed = 180f;
            AngularAcceleration = 360f;
            CurrentAngularVelocity = 0f;

            CurrentAngle = GetBaseAngle(orientation);
            _targetAngle = CurrentAngle;

            PositionX = (gridX + 0.5f) * cellSizeMeters;
            PositionY = (gridY + 0.5f) * cellSizeMeters;
            _startPhysX = PositionX;
            _startPhysY = PositionY;
            _targetPhysX = PositionX;
            _targetPhysY = PositionY;
        }

        public static float GetBaseAngle(RobotOrientation orientation)
        {
            // 将正交朝向枚举转换为标准的度数值。
            switch (orientation)
            {
                case RobotOrientation.Right:
                    return 0f;
                case RobotOrientation.Down:
                    return 90f;
                case RobotOrientation.Left:
                    return 180f;
                case RobotOrientation.Up:
                    return 270f;
                default:
                    return 0f;
            }
        }

        public static Point GetDirectionVector(RobotOrientation orientation)
        {
            // 将正交朝向枚举转换为二维离散单位步进向量。
            switch (orientation)
            {
                case RobotOrientation.Right:
                    return new Point(1, 0);
                case RobotOrientation.Down:
                    return new Point(0, 1);
                case RobotOrientation.Left:
                    return new Point(-1, 0);
                case RobotOrientation.Up:
                    return new Point(0, -1);
                default:
                    return Point.Empty;
            }
        }

        public bool TrySetAlgorithm(PathAlgorithm algorithm)
        {
            // 只有在到达目标点处于空闲状态时才允许更改寻路算法。
            // algorithm: 准备更改采用的目标寻路算法。
            if (NavState != RobotNavState.Idle)
            {
                return false;
            }

            CurrentAlgorithm = algorithm;
            return true;
        }

        public bool SetNavigationTarget(
            Point newTarget,
            int gridColumns,
            int gridRows,
            float cellSizeMeters,
            Func<Point, bool> isWalkable = null)
        {
            // 设置目标点或在移动状态下随时更换目标点并重新规划导航路径。
            // newTarget: 用户新指定的目标网格坐标。
            // gridColumns: 地图网格总列数。
            // gridRows: 地图网格总行数。
            // isWalkable: 判定网格是否可通行的回调委托。
            if (newTarget.X < 0 || newTarget.X >= gridColumns || newTarget.Y < 0 || newTarget.Y >= gridRows)
            {
                return false;
            }

            NavigationTarget = newTarget;

            Point logicalStart;
            bool wasMoving = (ActionState == RobotActionState.Moving);

            if (wasMoving)
            {
                // 若正在直线位移中，以符合最大加速度平滑减速刹停的安全网格为规划起点
                logicalStart = CalculateSafeStoppingGrid(cellSizeMeters, out int stopStepIndex);
                RedirectCurrentMoveToNextGrid(logicalStart, cellSizeMeters, stopStepIndex);
            }
            else
            {
                // 静止或正在原地转弯，直接以当前所在网格为规划起点
                logicalStart = new Point(GridX, GridY);
            }

            _pathQueue.Clear();
            CurrentPath.Clear();

            // 若规划起点恰好为新目标点
            if (logicalStart == newTarget)
            {
                if (wasMoving)
                {
                    StopAtNextGrid(cellSizeMeters);
                }
                else
                {
                    UpdateNavState(RobotNavState.Idle);
                    TargetReached?.Invoke();
                }
                return true;
            }

            // 以 logicalStart 为起点规划通往 newTarget 的无斜线正交路径
            List<Point> newPath = PathPlanning.FindPath(logicalStart, newTarget, gridColumns, gridRows, CurrentAlgorithm, isWalkable);
            if (newPath == null || newPath.Count == 0)
            {
                CancelNavigation();
                return false;
            }

            if (wasMoving)
            {
                for (int i = 1; i < newPath.Count; i++)
                {
                    _pathQueue.Enqueue(newPath[i]);
                }

                CurrentPath = new List<Point>(newPath);
            }
            else
            {
                for (int i = 1; i < newPath.Count; i++)
                {
                    _pathQueue.Enqueue(newPath[i]);
                }
                CurrentPath = new List<Point>(newPath);
            }

            UpdateNavState(RobotNavState.AutoMoving);

            if (ActionState == RobotActionState.Idle)
            {
                StepNavigation(gridColumns, gridRows, cellSizeMeters);
            }

            return true;
        }

        public bool AssignPlannedPath(
            List<Point> path,
            Point targetGrid,
            float cellSizeMeters,
            int gridColumns = 30,
            int gridRows = 30)
        {
            // 为机器人直接分配多机调度算法预先规划好的无冲突正交路径并启动导航。
            // path: 多机调度算法生成的离散栅格时空路径序列。
            // targetGrid: 机器人的目标网格点。
            // cellSizeMeters: 单个网格物理边长（米）。
            // gridColumns: 地图网格总列数。
            // gridRows: 地图网格总行数。
            if (path == null || path.Count == 0)
            {
                return false;
            }

            NavigationTarget = targetGrid;
            _pathQueue.Clear();
            CurrentPath.Clear();

            // 若起点已经处于终点
            if (path.Count == 1 && path[0] == targetGrid)
            {
                UpdateNavState(RobotNavState.Idle);
                TargetReached?.Invoke();
                return true;
            }

            // 将后续待行进的时空栅格节点依次入队
            for (int i = 1; i < path.Count; i++)
            {
                _pathQueue.Enqueue(path[i]);
            }
            CurrentPath = new List<Point>(path);

            UpdateNavState(RobotNavState.AutoMoving);

            // 外部调度接管：仅装载路径并挂起，由调度器授予首个受控区域许可后再起步
            if (ExternalScheduling)
            {
                IsAwaitingPermission = true;
            }
            else if (ActionState == RobotActionState.Idle)
            {
                StepNavigation(gridColumns, gridRows, cellSizeMeters);
            }

            return true;
        }

        private Point CalculateSafeStoppingGrid(float cellSizeMeters, out int stopStepIndex)
        {
            // 根据当前速度与设定加速度上限计算平滑制动停下所需的最小网格位置。
            // stopStepIndex: 相对于当前直线段起点的安全步数输出。
            Point dir = GetDirectionVector(Orientation);

            float currentDist = 0f;
            if (_moveDistanceMeters > 0.0001f)
            {
                float dx = PositionX - _startPhysX;
                float dy = PositionY - _startPhysY;
                currentDist = (float)Math.Sqrt(dx * dx + dy * dy);
            }

            float currentV = Math.Max(0.1f, CurrentSpeed);
            float safeAccel = Math.Max(0.1f, Acceleration);

            // 依据设定加速度上限 a 计算制动至 0 所需的最小物理位移
            float minBrakeDist = (currentV * currentV) / (2.0f * safeAccel);
            float minTargetDist = currentDist + minBrakeDist;

            // 换算为相对于直线起点的栅格步数索引（向上取整，且至少向前走一格）
            int requiredStep = (int)Math.Ceiling(minTargetDist / cellSizeMeters);
            stopStepIndex = Math.Max(_segmentStepsDequeued + 1, requiredStep);

            // 不超过当前直线段原本的总步长范围
            if (stopStepIndex > _segmentTotalSteps)
            {
                stopStepIndex = _segmentTotalSteps;
            }

            return new Point(
                _segmentStartGrid.X + dir.X * stopStepIndex,
                _segmentStartGrid.Y + dir.Y * stopStepIndex);
        }

        private void RedirectCurrentMoveToNextGrid(Point nextGrid, float cellSizeMeters, int stopStepIndex)
        {
            // 将当前进行中的直线移动目标截断重定向至下一个安全栅格以承接新规划路径。
            TargetGridX = nextGrid.X;
            TargetGridY = nextGrid.Y;

            float targetCenterX = (TargetGridX + 0.5f) * cellSizeMeters;
            float targetCenterY = (TargetGridY + 0.5f) * cellSizeMeters;

            _startPhysX = PositionX;
            _startPhysY = PositionY;
            _targetPhysX = targetCenterX;
            _targetPhysY = targetCenterY;

            float remainDist = (float)Math.Sqrt(
                (targetCenterX - PositionX) * (targetCenterX - PositionX) +
                (targetCenterY - PositionY) * (targetCenterY - PositionY));

            _moveDistanceMeters = Math.Max(0.005f, remainDist);
            _isBrakingToStop = true;
            _brakeInitialSpeed = Math.Max(0.2f, CurrentSpeed);
            _brakeDistance = _moveDistanceMeters;
            _brakeTotalTime = (2.0f * _brakeDistance) / _brakeInitialSpeed;
            _moveElapsed = 0f;

            _segmentTotalSteps = stopStepIndex;
        }

        public void StopAtNextGrid(float cellSizeMeters)
        {
            // 在按下停止时以不超过设定加速度的平滑减速度在安全栅格处停下。
            _pathQueue.Clear();
            CurrentPath.Clear();
            NavigationTarget = null;
            IsAwaitingPermission = false;

            if (ActionState != RobotActionState.Moving)
            {
                UpdateNavState(RobotNavState.Idle);
                return;
            }

            Point safeStopGrid = CalculateSafeStoppingGrid(cellSizeMeters, out int stopStepIndex);
            TargetGridX = safeStopGrid.X;
            TargetGridY = safeStopGrid.Y;

            float targetCenterX = (TargetGridX + 0.5f) * cellSizeMeters;
            float targetCenterY = (TargetGridY + 0.5f) * cellSizeMeters;

            _startPhysX = PositionX;
            _startPhysY = PositionY;
            _targetPhysX = targetCenterX;
            _targetPhysY = targetCenterY;

            float remainDist = (float)Math.Sqrt(
                (targetCenterX - PositionX) * (targetCenterX - PositionX) +
                (targetCenterY - PositionY) * (targetCenterY - PositionY));

            if (remainDist <= 0.005f)
            {
                PositionX = targetCenterX;
                PositionY = targetCenterY;
                GridX = TargetGridX;
                GridY = TargetGridY;
                CurrentSpeed = 0f;
                ActionState = RobotActionState.Idle;
                UpdateNavState(RobotNavState.Idle);
                return;
            }

            _isBrakingToStop = true;
            _brakeInitialSpeed = Math.Max(0.2f, CurrentSpeed);
            _brakeDistance = remainDist;
            _brakeTotalTime = (2.0f * _brakeDistance) / _brakeInitialSpeed;
            _moveElapsed = 0f;
            UpdateNavState(RobotNavState.Braking);
        }

        public void CancelNavigation()
        {
            // 取消当前自动导航任务并恢复空闲状态。
            _pathQueue.Clear();
            CurrentPath.Clear();
            NavigationTarget = null;

            if (ActionState == RobotActionState.Moving)
            {
                StopAtNextGrid(0.55f);
            }
            else
            {
                UpdateNavState(RobotNavState.Idle);
            }
        }

        private void UpdateNavState(RobotNavState newState)
        {
            // 更新高层导航状态机状态并触发状态变更通知事件。
            // newState: 新的导航状态机状态。
            if (NavState != newState)
            {
                NavState = newState;
                NavStateChanged?.Invoke(NavState);
            }
        }

        public bool TryMoveForward(int gridColumns, int gridRows, float cellSizeMeters)
        {
            // 手动请求向当前正交朝向前进一格。
            if (ActionState != RobotActionState.Idle)
            {
                return false;
            }

            int nextX = GridX;
            int nextY = GridY;

            switch (Orientation)
            {
                case RobotOrientation.Up:
                    nextY -= 1;
                    break;
                case RobotOrientation.Right:
                    nextX += 1;
                    break;
                case RobotOrientation.Down:
                    nextY += 1;
                    break;
                case RobotOrientation.Left:
                    nextX -= 1;
                    break;
            }

            if (nextX < 0 || nextX >= gridColumns || nextY < 0 || nextY >= gridRows)
            {
                return false;
            }

            _segmentStartGrid = new Point(GridX, GridY);
            _segmentTotalSteps = 1;
            _segmentStepsDequeued = 0;
            _isBrakingToStop = false;

            UpdateNavState(RobotNavState.ManualMoving);
            return TryStartStraightMove(new Point(nextX, nextY), 1, cellSizeMeters);
        }

        public bool TryTurnLeft()
        {
            // 请求机器人逆时针原地旋转90度。
            if (ActionState != RobotActionState.Idle)
            {
                return false;
            }

            UpdateNavState(RobotNavState.ManualMoving);
            RobotOrientation newOrientation = (RobotOrientation)(((int)Orientation - 1 + 4) % 4);
            StartTurning(newOrientation, -90f);
            return true;
        }

        public bool TryTurnRight()
        {
            // 请求机器人顺时针原地旋转90度。
            if (ActionState != RobotActionState.Idle)
            {
                return false;
            }

            UpdateNavState(RobotNavState.ManualMoving);
            RobotOrientation newOrientation = (RobotOrientation)(((int)Orientation + 1) % 4);
            StartTurning(newOrientation, 90f);
            return true;
        }

        public bool TryTurn180()
        {
            // 请求机器人一次性掉头旋转 180 度。
            if (ActionState != RobotActionState.Idle)
            {
                return false;
            }

            UpdateNavState(RobotNavState.ManualMoving);
            RobotOrientation newOrientation = (RobotOrientation)(((int)Orientation + 2) % 4);
            StartTurning(newOrientation, 180f);
            return true;
        }

        private void StartTurning(RobotOrientation newOrientation, float deltaAngle)
        {
            // 基于设定的角加速度与最大角速度规划二轮驱动机器人的平滑角速度动力学曲线。
            Orientation = newOrientation;
            _startAngle = CurrentAngle;
            _targetAngle = CurrentAngle + deltaAngle;
            _turnElapsed = 0f;

            _turnTotalAngle = Math.Abs(deltaAngle);
            _turnDirectionSign = Math.Sign(deltaAngle);

            float safeAlpha = Math.Max(10f, AngularAcceleration);
            float safeOmegaMax = Math.Max(10f, MaxAngularSpeed);

            // 加速到最大角速度所需的旋转角度 theta_acc = omega^2 / (2 * alpha)
            float accelAngle = (safeOmegaMax * safeOmegaMax) / (2.0f * safeAlpha);

            if (accelAngle * 2.0f >= _turnTotalAngle)
            {
                // 三角形角速度曲线：未达到最大角速度即开始减速，角加速度恒定为设定值
                _turnPeakSpeed = (float)Math.Sqrt(safeAlpha * _turnTotalAngle);
                _turnAccelTime = _turnPeakSpeed / safeAlpha;
                _turnConstTime = 0f;
                _turnDuration = _turnAccelTime * 2.0f;
            }
            else
            {
                // 梯形角速度曲线：角加速 -> 最大角速度匀速旋转 -> 角减速
                _turnPeakSpeed = safeOmegaMax;
                _turnAccelTime = _turnPeakSpeed / safeAlpha;
                float constAngle = _turnTotalAngle - accelAngle * 2.0f;
                _turnConstTime = constAngle / _turnPeakSpeed;
                _turnDuration = _turnAccelTime * 2.0f + _turnConstTime;
            }

            ActionState = RobotActionState.Turning;
        }

        private Point GetStraightLineTarget(Point currentGrid, RobotOrientation orientation, out int straightStepCount)
        {
            // 探测沿当前朝向可连续直行的最大网格数与终点网格（连续直行合批以获得完整梯形速度曲线）。
            straightStepCount = 0;
            Point lastStraightPoint = currentGrid;
            Point prevPoint = currentGrid;

            foreach (Point pt in _pathQueue)
            {
                if (pt == prevPoint)
                {
                    continue;
                }

                RobotOrientation stepOri = DetermineOrientation(prevPoint, pt);
                if (stepOri == orientation)
                {
                    straightStepCount++;
                    lastStraightPoint = pt;
                    prevPoint = pt;

                    // 外部调度接管时只走已获预约的前缀格数，绝不驶入未预约栅格
                    if (ExternalScheduling && _grantedStepLimit > 0 && straightStepCount >= _grantedStepLimit)
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
            }

            return lastStraightPoint;
        }

        /// <summary>
        /// 预览下一段动作涉及的全部受控栅格，供调度器一次性原子预约。
        /// 原地转向段返回当前所在格（转向时车身扫掠范围会略微探入相邻格，必须独占本格）；
        /// 直行段返回该段将连续驶入的全部栅格。空列表表示已无待行进路径。
        /// </summary>
        public List<Point> PeekSegmentZones()
        {
            List<Point> zones = new List<Point>();
            Point current = new Point(GridX, GridY);

            Point next = Point.Empty;
            bool hasNext = false;
            foreach (Point pt in _pathQueue)
            {
                if (pt != current)
                {
                    next = pt;
                    hasNext = true;
                    break;
                }
            }

            if (!hasNext)
            {
                return zones;
            }

            // 需要先转向：原地旋转不驶入新栅格，但必须独占当前格
            if (DetermineOrientation(current, next) != Orientation)
            {
                zones.Add(current);
                return zones;
            }

            Point prev = current;
            foreach (Point pt in _pathQueue)
            {
                if (pt == prev)
                {
                    continue;
                }
                if (DetermineOrientation(prev, pt) != Orientation)
                {
                    break;
                }
                zones.Add(pt);
                prev = pt;
            }

            return zones;
        }

        /// <summary>
        /// 当前合批直线段的真实起点与终点，供调度器判定哪些前方栅格尚未通过、不可提前释放。
        /// 直接采用段端点而非实时四舍五入的 GridX/Y，避免车身接近终点时方向向量退化导致误释放。
        /// </summary>
        public Point SegmentStartGrid => _segmentStartGrid;
        public Point SegmentEndGrid => new Point(TargetGridX, TargetGridY);

        /// <summary>
        /// 由调度器在受控区域预约成功后调用，放行机器人按完整梯形/三角形动力学执行移动。
        /// grantedSteps: 本次已预约的连续直行格数（0 表示原地转向段或单机模式不限）。
        /// </summary>
        public void GrantStepPermission(int gridColumns, int gridRows, float cellSizeMeters, int grantedSteps = 0)
        {
            if (!IsAwaitingPermission)
            {
                return;
            }

            IsAwaitingPermission = false;
            _grantedStepLimit = ExternalScheduling ? grantedSteps : 0;
            StepNavigation(gridColumns, gridRows, cellSizeMeters);
        }

        private bool TryStartStraightMove(Point endGrid, int steps, float cellSizeMeters)
        {
            // 启动跨越连续同向直线的平滑梯形加减速移动。
            // 核心安全约束：严格依据机器人当前正交朝向推进计算目标网格，从几何根源杜绝对角斜线位移。
            Point dirVec = GetDirectionVector(Orientation);
            int safeSteps = Math.Max(1, steps);
            TargetGridX = GridX + dirVec.X * safeSteps;
            TargetGridY = GridY + dirVec.Y * safeSteps;

            _startPhysX = (GridX + 0.5f) * cellSizeMeters;
            _startPhysY = (GridY + 0.5f) * cellSizeMeters;
            _targetPhysX = (TargetGridX + 0.5f) * cellSizeMeters;
            _targetPhysY = (TargetGridY + 0.5f) * cellSizeMeters;

            _moveDistanceMeters = safeSteps * cellSizeMeters;
            _moveElapsed = 0f;
            _isBrakingToStop = false;

            float safeAccel = Math.Max(0.1f, Acceleration);
            float safeMaxSpeed = Math.Max(0.1f, MaxSpeed);

            float accelDist = (safeMaxSpeed * safeMaxSpeed) / (2.0f * safeAccel);
            if (accelDist * 2.0f >= _moveDistanceMeters)
            {
                _peakSpeed = (float)Math.Sqrt(safeAccel * _moveDistanceMeters);
                _accelTime = _peakSpeed / safeAccel;
                _constTime = 0f;
                _totalMoveTime = _accelTime * 2.0f;
            }
            else
            {
                _peakSpeed = safeMaxSpeed;
                _accelTime = _peakSpeed / safeAccel;
                float constDist = _moveDistanceMeters - accelDist * 2.0f;
                _constTime = constDist / _peakSpeed;
                _totalMoveTime = _accelTime * 2.0f + _constTime;
            }

            ActionState = RobotActionState.Moving;
            return true;
        }

        private void StepNavigation(int gridColumns, int gridRows, float cellSizeMeters)
        {
            // 探测前方连续直线或旋转需求并推进导航。
            if (_pathQueue.Count == 0)
            {
                bool isRealTargetArrival = NavigationTarget.HasValue &&
                                          GridX == NavigationTarget.Value.X &&
                                          GridY == NavigationTarget.Value.Y;
                NavigationTarget = null;
                CurrentPath.Clear();
                UpdateNavState(RobotNavState.Idle);
                if (isRealTargetArrival)
                {
                    TargetReached?.Invoke();
                }
                return;
            }

            // 过滤并弹出队列头部与当前所在格重叠的点
            while (_pathQueue.Count > 0 && _pathQueue.Peek().X == GridX && _pathQueue.Peek().Y == GridY)
            {
                _pathQueue.Dequeue();
                CurrentPath = new List<Point>(_pathQueue);
            }

            if (_pathQueue.Count == 0)
            {
                NavigationTarget = null;
                CurrentPath.Clear();
                UpdateNavState(RobotNavState.Idle);
                TargetReached?.Invoke();
                return;
            }

            Point nextPoint = _pathQueue.Peek();
            RobotOrientation neededOrientation = DetermineOrientation(new Point(GridX, GridY), nextPoint);

            if (neededOrientation != Orientation)
            {
                // 朝向不一致时一次性旋转至目标方向（支持 90 度转弯或 180 度掉头）
                int diff = ((int)neededOrientation - (int)Orientation + 4) % 4;
                if (diff == 1)
                {
                    StartTurning(neededOrientation, 90f);
                }
                else if (diff == 3)
                {
                    StartTurning(neededOrientation, -90f);
                }
                else if (diff == 2)
                {
                    StartTurning(neededOrientation, 180f);
                }
            }
            else
            {
                // 朝向已一致：探测前方连续同向直线的最大步数，以最大速度匀速行驶
                Point straightEnd = GetStraightLineTarget(new Point(GridX, GridY), Orientation, out int steps);
                if (steps <= 0)
                {
                    steps = 1;
                    straightEnd = nextPoint;
                }

                _segmentStartGrid = new Point(GridX, GridY);
                _segmentTotalSteps = steps;
                _segmentStepsDequeued = 0;
                _isBrakingToStop = false;

                // 保持完整路径供以栅格为单位实时显示，不在起跑瞬间整段出队
                CurrentPath = new List<Point>(_pathQueue);
                TryStartStraightMove(straightEnd, steps, cellSizeMeters);
            }
        }

        private void CompleteSegmentAndAwait(int gridColumns, int gridRows, float cellSizeMeters)
        {
            // 一段移动结束后的续接决策：外部调度接管时必须挂起等待许可，否则按原有逻辑自行续接。
            if (ExternalScheduling)
            {
                if (_pathQueue.Count > 0)
                {
                    IsAwaitingPermission = true;
                }
                else
                {
                    bool arrived = NavigationTarget.HasValue &&
                                   GridX == NavigationTarget.Value.X &&
                                   GridY == NavigationTarget.Value.Y;
                    NavigationTarget = null;
                    CurrentPath.Clear();
                    UpdateNavState(RobotNavState.Idle);
                    if (arrived)
                    {
                        TargetReached?.Invoke();
                    }
                }
                return;
            }

            if (NavState == RobotNavState.AutoMoving)
            {
                StepNavigation(gridColumns, gridRows, cellSizeMeters);
            }
            else
            {
                UpdateNavState(RobotNavState.Idle);
            }
        }

        private static RobotOrientation DetermineOrientation(Point from, Point to)
        {
            // 计算从源网格节点移动至相邻目标网格节点所需的朝向。
            // from: 起始相邻网格。
            // to: 目标相邻网格。
            if (to.X > from.X) return RobotOrientation.Right;
            if (to.X < from.X) return RobotOrientation.Left;
            if (to.Y > from.Y) return RobotOrientation.Down;
            return RobotOrientation.Up;
        }

        public void ResetToGrid(int gridX, int gridY, float cellSizeMeters)
        {
            // 将机器人瞬移重置回指定网格中心并清空所有状态。
            _pathQueue.Clear();
            CurrentPath.Clear();
            NavigationTarget = null;
            _isBrakingToStop = false;
            IsAwaitingPermission = false;

            GridX = gridX;
            GridY = gridY;
            TargetGridX = gridX;
            TargetGridY = gridY;
            Orientation = RobotOrientation.Up;
            ActionState = RobotActionState.Idle;
            UpdateNavState(RobotNavState.Idle);

            CurrentSpeed = 0f;
            CurrentAngle = GetBaseAngle(Orientation);
            _targetAngle = CurrentAngle;

            PositionX = (gridX + 0.5f) * cellSizeMeters;
            PositionY = (gridY + 0.5f) * cellSizeMeters;
            _startPhysX = PositionX;
            _startPhysY = PositionY;
            _targetPhysX = PositionX;
            _targetPhysY = PositionY;
        }

        public void Update(float deltaTime, float cellSizeMeters, int gridColumns = 30, int gridRows = 30)
        {
            // 根据时间流逝更新机器人的位置插值、转向插值及导航推进。
            // deltaTime: 自上一帧以来的时间步长（秒）。
            if (deltaTime <= 0f)
            {
                return;
            }

            if (ActionState == RobotActionState.Turning)
            {
                _turnElapsed += deltaTime;
                if (_turnElapsed >= _turnDuration)
                {
                    CurrentAngle = GetBaseAngle(Orientation);
                    _targetAngle = CurrentAngle;
                    CurrentAngularVelocity = 0f;
                    ActionState = RobotActionState.Idle;

                    CompleteSegmentAndAwait(gridColumns, gridRows, cellSizeMeters);
                }
                else
                {
                    float t = _turnElapsed;
                    float rotatedAngle = 0f;
                    float safeAlpha = Math.Max(10f, AngularAcceleration);

                    if (t < _turnAccelTime)
                    {
                        // 1. 角加速阶段: 角加速度恒为 safeAlpha，角速度从 0 线性上升
                        rotatedAngle = 0.5f * safeAlpha * t * t;
                        CurrentAngularVelocity = safeAlpha * t;
                    }
                    else if (t < _turnAccelTime + _turnConstTime)
                    {
                        // 2. 匀角速阶段: 保持最大角速度匀速旋转
                        float accelAngle = 0.5f * safeAlpha * _turnAccelTime * _turnAccelTime;
                        float constElapsed = t - _turnAccelTime;
                        rotatedAngle = accelAngle + _turnPeakSpeed * constElapsed;
                        CurrentAngularVelocity = _turnPeakSpeed;
                    }
                    else
                    {
                        // 3. 角减速阶段: 恒定角减速，角速度线性归零
                        float decelElapsed = t - (_turnAccelTime + _turnConstTime);
                        float remainTime = _turnAccelTime - decelElapsed;
                        if (remainTime < 0f) remainTime = 0f;

                        float accelAngle = 0.5f * safeAlpha * _turnAccelTime * _turnAccelTime;
                        float constAngle = _turnPeakSpeed * _turnConstTime;
                        float decelCovered = (0.5f * safeAlpha * _turnAccelTime * _turnAccelTime) - (0.5f * safeAlpha * remainTime * remainTime);

                        rotatedAngle = accelAngle + constAngle + decelCovered;
                        CurrentAngularVelocity = safeAlpha * remainTime;
                    }

                    CurrentAngle = _startAngle + (_turnDirectionSign * rotatedAngle);
                }
            }
            else if (ActionState == RobotActionState.Moving)
            {
                CurrentAngularVelocity = 0f;
                _moveElapsed += deltaTime;

                if (_isBrakingToStop)
                {
                    // 在下一个相邻栅格处执行平滑减速刹车
                    if (_moveElapsed >= _brakeTotalTime)
                    {
                        PositionX = _targetPhysX;
                        PositionY = _targetPhysY;
                        GridX = TargetGridX;
                        GridY = TargetGridY;
                        CurrentSpeed = 0f;
                        ActionState = RobotActionState.Idle;
                        _isBrakingToStop = false;

                        if (ExternalScheduling)
                        {
                            if (_pathQueue.Count > 0)
                            {
                                IsAwaitingPermission = true;
                            }
                            else
                            {
                                NavigationTarget = null;
                                CurrentPath.Clear();
                                UpdateNavState(RobotNavState.Idle);
                            }
                        }
                        else if (NavState == RobotNavState.AutoMoving && _pathQueue.Count > 0)
                        {
                            StepNavigation(gridColumns, gridRows, cellSizeMeters);
                        }
                        else
                        {
                            bool isRealTargetArrival = NavigationTarget.HasValue &&
                                                      GridX == NavigationTarget.Value.X &&
                                                      GridY == NavigationTarget.Value.Y;
                            NavigationTarget = null;
                            UpdateNavState(RobotNavState.Idle);
                            if (isRealTargetArrival)
                            {
                                TargetReached?.Invoke();
                            }
                        }
                    }
                    else
                    {
                        float t = _moveElapsed;
                        float ratio = (t / _brakeTotalTime);
                        float s = _brakeInitialSpeed * t - 0.5f * (_brakeInitialSpeed / _brakeTotalTime) * t * t;
                        if (s > _brakeDistance) s = _brakeDistance;
                        float moveRatio = (_brakeDistance > 0.0001f) ? (s / _brakeDistance) : 1f;

                        PositionX = _startPhysX + (_targetPhysX - _startPhysX) * moveRatio;
                        PositionY = _startPhysY + (_targetPhysY - _startPhysY) * moveRatio;
                        CurrentSpeed = Math.Max(0f, _brakeInitialSpeed * (1f - ratio));

                        GridX = (int)Math.Round((PositionX / cellSizeMeters) - 0.5f);
                        GridY = (int)Math.Round((PositionY / cellSizeMeters) - 0.5f);
                    }
                    return;
                }

                if (_moveElapsed >= _totalMoveTime)
                {
                    // 直线段跑完，将剩余该段的点全部出队
                    while (_segmentStepsDequeued < _segmentTotalSteps && _pathQueue.Count > 0)
                    {
                        _pathQueue.Dequeue();
                        _segmentStepsDequeued++;
                    }
                    CurrentPath = new List<Point>(_pathQueue);

                    GridX = TargetGridX;
                    GridY = TargetGridY;
                    PositionX = (GridX + 0.5f) * cellSizeMeters;
                    PositionY = (GridY + 0.5f) * cellSizeMeters;
                    CurrentSpeed = 0f;
                    ActionState = RobotActionState.Idle;

                    CompleteSegmentAndAwait(gridColumns, gridRows, cellSizeMeters);
                }
                else
                {
                    float s = 0f;
                    float safeAccel = Math.Max(0.1f, Acceleration);

                    if (_moveElapsed < _accelTime)
                    {
                        s = 0.5f * safeAccel * _moveElapsed * _moveElapsed;
                        CurrentSpeed = safeAccel * _moveElapsed;
                    }
                    else if (_moveElapsed < _accelTime + _constTime)
                    {
                        float accelDist = 0.5f * safeAccel * _accelTime * _accelTime;
                        float constElapsed = _moveElapsed - _accelTime;
                        s = accelDist + _peakSpeed * constElapsed;
                        CurrentSpeed = _peakSpeed;
                    }
                    else
                    {
                        float decelElapsed = _moveElapsed - (_accelTime + _constTime);
                        float remainingTime = _accelTime - decelElapsed;
                        if (remainingTime < 0f) remainingTime = 0f;

                        float accelDist = 0.5f * safeAccel * _accelTime * _accelTime;
                        float constDist = _peakSpeed * _constTime;
                        float decelDistCovered = (0.5f * safeAccel * _accelTime * _accelTime) - (0.5f * safeAccel * remainingTime * remainingTime);

                        s = accelDist + constDist + decelDistCovered;
                        CurrentSpeed = safeAccel * remainingTime;
                    }

                    float ratio = (_moveDistanceMeters > 0.0001f) ? (s / _moveDistanceMeters) : 1f;
                    if (ratio > 1f) ratio = 1f;

                    PositionX = _startPhysX + (_targetPhysX - _startPhysX) * ratio;
                    PositionY = _startPhysY + (_targetPhysY - _startPhysY) * ratio;

                    GridX = (int)Math.Round((PositionX / cellSizeMeters) - 0.5f);
                    GridY = (int)Math.Round((PositionY / cellSizeMeters) - 0.5f);

                    // 随物理位移逐格弹出已通过的路径点，使路径显示与剩余步数实时同步。
                    // 授权已改为整段原子预约，调度器不再依赖 CurrentPath[0]，故此处出队安全。
                    int passedSteps = (int)Math.Floor((s + 0.001f) / cellSizeMeters);
                    if (passedSteps > _segmentStepsDequeued)
                    {
                        while (_segmentStepsDequeued < passedSteps && _pathQueue.Count > 0)
                        {
                            _pathQueue.Dequeue();
                            _segmentStepsDequeued++;
                        }
                        CurrentPath = new List<Point>(_pathQueue);
                    }
                }
            }
            else
            {
                CurrentSpeed = 0f;
                CurrentAngularVelocity = 0f;
                PositionX = (GridX + 0.5f) * cellSizeMeters;
                PositionY = (GridY + 0.5f) * cellSizeMeters;

                // 外部调度接管时严禁自行续接，必须等待调度器授予通行许可
                if (!ExternalScheduling && NavState == RobotNavState.AutoMoving && _pathQueue.Count > 0)
                {
                    StepNavigation(gridColumns, gridRows, cellSizeMeters);
                }
            }
        }

        public void Render(
            SKCanvas canvas,
            float offsetX,
            float offsetY,
            float scale,
            float baseCellPixels,
            float cellSizeMeters)
        {
            // 使用 SkiaSharp 高质量向量渲染机器人底盘、车头箭头与中心枢轴。
            // canvas: Skia 绘图画布。
            // offsetX: 视口 X 轴像素偏移。
            // offsetY: 视口 Y 轴像素偏移。
            // scale: 视口缩放系数。
            // baseCellPixels: 基础网格像素大小。
            float pixelsPerMeter = (baseCellPixels * scale) / cellSizeMeters;
            float screenX = offsetX + PositionX * pixelsPerMeter;
            float screenY = offsetY + PositionY * pixelsPerMeter;

            float robotPixelWidth = ChassisWidthMeters * pixelsPerMeter;
            float robotPixelLength = ChassisLengthMeters * pixelsPerMeter;

            canvas.Save();
            canvas.Translate(screenX, screenY);
            canvas.RotateDegrees(CurrentAngle);

            SKRect chassisRect = new SKRect(
                -robotPixelLength * 0.5f,
                -robotPixelWidth * 0.5f,
                robotPixelLength * 0.5f,
                robotPixelWidth * 0.5f);

            float cornerRadius = Math.Max(2f, robotPixelWidth * 0.12f);

            using (var shadowPaint = new SKPaint
            {
                Color = new SKColor(0, 0, 0, 50),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                SKRect shadowRect = chassisRect;
                shadowRect.Offset(2f * scale, 2f * scale);
                canvas.DrawRoundRect(shadowRect, cornerRadius, cornerRadius, shadowPaint);
            }

            using (var bodyPaint = new SKPaint
            {
                Color = new SKColor(255, 175, 20),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                canvas.DrawRoundRect(chassisRect, cornerRadius, cornerRadius, bodyPaint);
            }

            using (var strokePaint = new SKPaint
            {
                Color = new SKColor(35, 45, 55),
                IsAntialias = true,
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(1.5f, 2f * scale)
            })
            {
                canvas.DrawRoundRect(chassisRect, cornerRadius, cornerRadius, strokePaint);
            }

            float arrowLen = robotPixelLength * 0.40f;
            float arrowWid = robotPixelWidth * 0.45f;
            using (var arrowPath = new SKPath())
            using (var arrowPaint = new SKPaint
            {
                Color = new SKColor(235, 45, 45),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                arrowPath.MoveTo(robotPixelLength * 0.45f, 0f);
                arrowPath.LineTo(robotPixelLength * 0.45f - arrowLen, -arrowWid * 0.5f);
                arrowPath.LineTo(robotPixelLength * 0.45f - arrowLen, arrowWid * 0.5f);
                arrowPath.Close();
                canvas.DrawPath(arrowPath, arrowPaint);
            }

            float centerRadius = Math.Max(2f, robotPixelWidth * 0.16f);
            using (var centerPaint = new SKPaint
            {
                Color = new SKColor(40, 50, 60),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                canvas.DrawCircle(0f, 0f, centerRadius, centerPaint);
            }

            canvas.Restore();
        }
    }
}