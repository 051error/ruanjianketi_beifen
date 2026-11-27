using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using SkiaSharp;
using WarehouseGridSimulation.Core.Engine;
using WarehouseGridSimulation.Core.Geometry;
using WarehouseGridSimulation.Core.Map;
using WarehouseGridSimulation.Core.Robots;

namespace WarehouseGridSimulation.Skia.Rendering
{
    /// <summary>
    /// 仿真运行模式。
    /// </summary>
    public enum SimulationMode
    {
        SingleRobot = 0,
        MultiRobot = 1
    }

    /// <summary>
    /// 基于 SkiaSharp 的仓储仿真渲染引擎。
    /// 负责高精度栅格地图、多机器人位姿底盘、路径轨迹、目标标记与双半透明 HUD 仪表卡片的光栅化渲染。
    /// </summary>
    public class SkiaWarehouseRenderer : IDisposable
    {
        private static SKTypeface _yaHeiTypeface;
        private static SKTypeface _yaHeiBoldTypeface;
        private readonly SKPaint _floorPaint = new SKPaint { Color = SKColors.White, Style = SKPaintStyle.Fill };
        private readonly SKPaint _gridPaint = new SKPaint { Color = new SKColor(220, 225, 230), Style = SKPaintStyle.Stroke, StrokeWidth = 1f };
        private readonly SKPaint _borderPaint = new SKPaint { Color = new SKColor(50, 62, 72), Style = SKPaintStyle.Stroke, StrokeWidth = 2f, IsAntialias = true };
        private readonly SKPaint _shadowPaint = new SKPaint { Color = new SKColor(0, 0, 0, 45), Style = SKPaintStyle.Fill, IsAntialias = true };
        private readonly SKPaint _bodyPaint = new SKPaint { Style = SKPaintStyle.Fill, IsAntialias = true };
        private readonly SKPaint _bodyStrokePaint = new SKPaint { Color = new SKColor(35, 45, 55), Style = SKPaintStyle.Stroke, StrokeWidth = 1.5f, IsAntialias = true };
        private readonly SKPaint _arrowPaint = new SKPaint { Color = new SKColor(235, 45, 45), Style = SKPaintStyle.Fill, IsAntialias = true };
        private readonly SKPaint _centerPaint = new SKPaint { Color = new SKColor(40, 50, 60), Style = SKPaintStyle.Fill, IsAntialias = true };
        private readonly SKPaint _badgeBackgroundPaint = new SKPaint { Color = new SKColor(35, 45, 55), Style = SKPaintStyle.Fill, IsAntialias = true };
        private readonly SKPaint _badgeCollisionPaint = new SKPaint { Color = new SKColor(220, 40, 40), Style = SKPaintStyle.Fill, IsAntialias = true };
        private readonly SKPaint _badgeTextPaint = new SKPaint { Color = SKColors.White, Typeface = YaHeiBoldTypeface, TextSize = 9.5f, IsAntialias = true, TextAlign = SKTextAlign.Center };
        private readonly SKPaint _selectionHaloPaint = new SKPaint { Color = new SKColor(0, 195, 255, 230), Style = SKPaintStyle.Stroke, StrokeWidth = 2.5f, IsAntialias = true };
        private readonly SKPath _arrowPath = new SKPath();
        private float _arrowBodyLength = -1f;
        private float _arrowBodyWidth = -1f;

        public static SKTypeface YaHeiTypeface => _yaHeiTypeface ??= SKTypeface.FromFamilyName("Microsoft YaHei");
        public static SKTypeface YaHeiBoldTypeface => _yaHeiBoldTypeface ??= SKTypeface.FromFamilyName("Microsoft YaHei", SKFontStyle.Bold);

        private static readonly SKColor RobotWarmYellowColor = new SKColor(255, 175, 20); // 专属暖黄色小车底盘
        private static readonly SKColor UnifiedPathColor = new SKColor(30, 144, 255);      // 统一规划路径折线色 (亮蓝)

        public void Render(
            SKCanvas canvas,
            int viewWidth,
            int viewHeight,
            GridMap map,
            SimulationSnapshot snapshot,
            float offsetX,
            float offsetY,
            float scale,
            float baseCellPixels,
            SimulationMode mode,
            System.Drawing.Point? selectedTarget,
            bool showPath,
            bool showTargetLines,
            IReadOnlyList<System.Drawing.Point> multiTargets,
            bool isViewportHudMinimized,
            bool isStatusHudMinimized,
            ref SKRect viewportHudRect,
            ref SKRect statusHudRect,
            ref SKRect viewportMinBtnRect,
            ref SKRect statusMinBtnRect,
            float renderFps,
            float interpolationAlpha,
            int? selectedRobotId = null,
            double multiElapsedTime = 0.0,
            int completedTasksCount = 0,
            double taskEfficiency = 0.0)
        {
            // 使用 SkiaSharp 统一光栅化渲染全场景地图、机群位姿、路径轨迹与双 HUD 仪表盘。
            // canvas: 渲染目标画布; viewWidth: 视口宽度; viewHeight: 视口高度; map: 仓储网格地图; snapshot: 最新权威状态快照; offsetX: 视口水平平移量; offsetY: 视口垂直平移量; scale: 视口缩放比例; baseCellPixels: 基础单格像素跨度; mode: 当前仿真运行模式; selectedTarget: 单机模式下选中的目标点坐标; showPath: 是否渲染规划路径折线; showTargetLines: 是否绘制目标配对连线; multiTargets: 多机模式下的目标点集合; isViewportHudMinimized: 视口 HUD 卡片是否处于最小化收起状态; isStatusHudMinimized: 状态 HUD 卡片是否处于最小化收起状态; viewportHudRect: 视口 HUD 卡片外框几何矩形引用; statusHudRect: 状态 HUD 卡片外框几何矩形引用; viewportMinBtnRect: 视口折叠按钮矩形引用; statusMinBtnRect: 状态折叠按钮矩形引用; renderFps: 当前界面实时渲染刷新帧率; interpolationAlpha: 渲染位姿插值系数; selectedRobotId: 当前选中受控的机器人全局编号; multiElapsedTime: 多机调度累计运行时长秒数; completedTasksCount: 多机模式已完成任务总数; taskEfficiency: 机群每分钟完成任务数锁存效率。
            if (canvas == null) return;

            // 1. 背景清屏 (柔和浅色背景)
            canvas.Clear(new SKColor(245, 247, 250));

            if (map == null) return;

            float cellPixels = baseCellPixels * scale;
            float gridWidth = map.Width * cellPixels;
            float gridHeight = map.Height * cellPixels;

            // 2. 绘制地图地坪与网格线
            DrawFloorAndGrid(canvas, map, offsetX, offsetY, cellPixels, gridWidth, gridHeight);

            // 3. 绘制资源预约与占用高亮
            DrawReservations(canvas, snapshot, offsetX, offsetY, cellPixels);

            // 4. 绘制障碍物
            DrawObstacles(canvas, map, offsetX, offsetY, cellPixels);

            // 5. 绘制仓储作业站点
            DrawStations(canvas, map, offsetX, offsetY, cellPixels);

            if (mode == SimulationMode.SingleRobot)
            {
                // 单机模式：绘制规划路径与选中目标标记
                if (showPath)
                {
                    DrawPlannedPaths(canvas, snapshot, map, offsetX, offsetY, cellPixels, interpolationAlpha);
                }
                DrawTargetMarker(canvas, selectedTarget, offsetX, offsetY, cellPixels, scale);
            }
            else
            {
                // 多机模式：绘制多机目标、连线与路径
                DrawMultiRobotTargets(canvas, multiTargets, offsetX, offsetY, cellPixels);
                if (showTargetLines && snapshot?.Robots != null)
                {
                    DrawMultiTargetLines(canvas, snapshot, multiTargets, map, offsetX, offsetY, cellPixels, interpolationAlpha);
                }
                if (showPath)
                {
                    DrawPlannedPaths(canvas, snapshot, map, offsetX, offsetY, cellPixels, interpolationAlpha);
                }
            }

            // 6. 绘制所有机器人实体
            DrawRobots(canvas, snapshot, map, offsetX, offsetY, cellPixels, interpolationAlpha, selectedRobotId);

            // 绘制机器人右上角编号徽标 (多机模式，或存在碰撞警示时显示红标)
            if (snapshot?.Robots != null && (mode == SimulationMode.MultiRobot || snapshot.Robots.Any(r => r.HasCollided)))
            {
                DrawMultiRobotBadges(canvas, snapshot, map, offsetX, offsetY, cellPixels, interpolationAlpha);
            }

            // 7. 绘制左上角视口说明 HUD 卡片
            DrawViewportHud(
                canvas,
                map,
                mode,
                snapshot,
                isViewportHudMinimized,
                ref viewportHudRect,
                ref viewportMinBtnRect);

            // 8. 绘制左下角实时状态监控 HUD 卡片
            DrawStatusMonitorHud(
                canvas,
                viewWidth,
                viewHeight,
                snapshot,
                mode,
                selectedTarget,
                multiTargets,
                isStatusHudMinimized,
                ref statusHudRect,
                ref statusMinBtnRect,
                renderFps,
                selectedRobotId,
                multiElapsedTime,
                completedTasksCount,
                taskEfficiency);
        }

        private void DrawFloorAndGrid(SKCanvas canvas, GridMap map, float ox, float oy, float cellPixels, float gw, float gh)
        {
            // 绘制地坪阴影、白色底衬与正交栅格分割细线。
            canvas.DrawRect(ox, oy, gw, gh, _floorPaint);
            for (int col = 0; col <= map.Width; col++)
            {
                float x = ox + col * cellPixels;
                canvas.DrawLine(x, oy, x, oy + gh, _gridPaint);
            }
            for (int row = 0; row <= map.Height; row++)
            {
                float y = oy + row * cellPixels;
                canvas.DrawLine(ox, oy + row * cellPixels, ox + gw, oy + row * cellPixels, _gridPaint);
            }
            canvas.DrawRect(ox, oy, gw, gh, _borderPaint);
        }

        private void DrawReservations(SKCanvas canvas, SimulationSnapshot snapshot, float ox, float oy, float cellPixels)
        {
            // 绘制受控区域中已被物理占用的栅格高亮。
            if (snapshot == null) return;

            if (snapshot.OccupiedGrids != null)
            {
                using (var occPaint = new SKPaint
                {
                    Color = new SKColor(255, 175, 45, 40),
                    Style = SKPaintStyle.Fill
                })
                {
                    foreach (var kvp in snapshot.OccupiedGrids)
                    {
                        float x = ox + kvp.Key.X * cellPixels;
                        float y = oy + kvp.Key.Y * cellPixels;
                        canvas.DrawRect(x, y, cellPixels, cellPixels, occPaint);
                    }
                }
            }
        }

        private void DrawObstacles(SKCanvas canvas, GridMap map, float ox, float oy, float cellPixels)
        {
            // 绘制地图中的立体货架障碍物实体。
            if (map.Obstacles.Count == 0) return;

            using (var shelfBg = new SKPaint
            {
                Color = new SKColor(90, 105, 120),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            using (var shelfBorder = new SKPaint
            {
                Color = new SKColor(60, 75, 90),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1f,
                IsAntialias = true
            })
            {
                foreach (var obs in map.Obstacles)
                {
                    float x = ox + obs.X * cellPixels + 1.5f;
                    float y = oy + obs.Y * cellPixels + 1.5f;
                    float w = cellPixels - 3f;
                    float h = cellPixels - 3f;

                    var rrect = new SKRoundRect(new SKRect(x, y, x + w, y + h), 3f);
                    canvas.DrawRoundRect(rrect, shelfBg);
                    canvas.DrawRoundRect(rrect, shelfBorder);
                }
            }
        }

        private void DrawStations(SKCanvas canvas, GridMap map, float ox, float oy, float cellPixels)
        {
            // 绘制上货、卸货与待命作业站点标记。
            foreach (var st in map.Stations)
            {
                float x = ox + st.Position.X * cellPixels;
                float y = oy + st.Position.Y * cellPixels;

                SKColor bgColor = st.Type switch
                {
                    StationType.Pick => new SKColor(46, 204, 113, 50),
                    StationType.Drop => new SKColor(231, 76, 60, 50),
                    StationType.Waiting => new SKColor(243, 156, 18, 50),
                    _ => new SKColor(243, 156, 18, 50)
                };

                SKColor borderColor = st.Type switch
                {
                    StationType.Pick => new SKColor(39, 174, 96),
                    StationType.Drop => new SKColor(192, 57, 43),
                    StationType.Waiting => new SKColor(230, 126, 34),
                    _ => new SKColor(230, 126, 34)
                };

                using (var bgPaint = new SKPaint { Color = bgColor, Style = SKPaintStyle.Fill, IsAntialias = true })
                using (var borderPaint = new SKPaint { Color = borderColor, Style = SKPaintStyle.Stroke, StrokeWidth = 1.5f, IsAntialias = true })
                {
                    var rect = new SKRoundRect(new SKRect(x + 2, y + 2, x + cellPixels - 2, y + cellPixels - 2), 3f);
                    canvas.DrawRoundRect(rect, bgPaint);
                    canvas.DrawRoundRect(rect, borderPaint);
                }
            }
        }

        private void DrawTargetMarker(SKCanvas canvas, System.Drawing.Point? selectedTarget, float ox, float oy, float cellPixels, float scale)
        {
            // 在用户选定的目标网格上绘制同心圆靶心标记。
            if (!selectedTarget.HasValue) return;

            var target = selectedTarget.Value;
            float cellX = ox + target.X * cellPixels;
            float cellY = oy + target.Y * cellPixels;
            float centerX = cellX + cellPixels * 0.5f;
            float centerY = cellY + cellPixels * 0.5f;

            using (var bgPaint = new SKPaint { Color = new SKColor(235, 60, 60, 50), Style = SKPaintStyle.Fill })
            using (var borderPaint = new SKPaint
            {
                Color = new SKColor(225, 40, 40),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(1.5f, 2f * scale),
                IsAntialias = true
            })
            {
                canvas.DrawRect(cellX, cellY, cellPixels, cellPixels, bgPaint);
                canvas.DrawRect(cellX, cellY, cellPixels, cellPixels, borderPaint);
            }

            float outerRadius = cellPixels * 0.35f;
            float innerRadius = cellPixels * 0.18f;
            float centerDotRadius = Math.Max(1.5f, cellPixels * 0.08f);

            using (var ringPaint = new SKPaint
            {
                Color = new SKColor(220, 30, 30),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = Math.Max(1f, 1.8f * scale),
                IsAntialias = true
            })
            using (var centerDotPaint = new SKPaint { Color = new SKColor(220, 30, 30), Style = SKPaintStyle.Fill, IsAntialias = true })
            {
                canvas.DrawCircle(centerX, centerY, outerRadius, ringPaint);
                canvas.DrawCircle(centerX, centerY, innerRadius, ringPaint);
                canvas.DrawCircle(centerX, centerY, centerDotRadius, centerDotPaint);
            }
        }

        private void DrawMultiRobotTargets(SKCanvas canvas, IReadOnlyList<System.Drawing.Point> multiTargets, float ox, float oy, float cellPixels)
        {
            // 绘制多机调度模式下分配给各机器人的专属目标标记。
            if (multiTargets == null || multiTargets.Count == 0) return;

            using (var fillPaint = new SKPaint { Color = new SKColor(160, 60, 220, 55), Style = SKPaintStyle.Fill })
            using (var borderPaint = new SKPaint { Color = new SKColor(140, 40, 200), Style = SKPaintStyle.Stroke, StrokeWidth = 1.5f, IsAntialias = true })
            using (var flagPaint = new SKPaint { Color = new SKColor(140, 40, 200), Style = SKPaintStyle.Fill, IsAntialias = true })
            {
                int count = multiTargets.Count;
                for (int i = 0; i < count; i++)
                {
                    var target = multiTargets[i];
                    float cellX = ox + target.X * cellPixels;
                    float cellY = oy + target.Y * cellPixels;
                    canvas.DrawRect(cellX, cellY, cellPixels, cellPixels, fillPaint);
                    canvas.DrawRect(cellX, cellY, cellPixels, cellPixels, borderPaint);

                    float cx = cellX + cellPixels * 0.5f;
                    float cy = cellY + cellPixels * 0.5f;
                    canvas.DrawCircle(cx, cy, cellPixels * 0.22f, flagPaint);
                }
            }
        }

        private void DrawMultiTargetLines(SKCanvas canvas, SimulationSnapshot snapshot, IReadOnlyList<System.Drawing.Point> multiTargets, GridMap map, float ox, float oy, float cellPixels, float interpolationAlpha)
        {
            // 在多机模式下绘制从机器人当前位置到其专属目标的指引连线。
            if (snapshot?.Robots == null || multiTargets == null) return;

            using (var linePaint = new SKPaint
            {
                Color = new SKColor(160, 90, 220, 80),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1f,
                PathEffect = SKPathEffect.CreateDash(new[] { 3f, 3f }, 0),
                IsAntialias = true
            })
            {
                for (int i = 0; i < snapshot.Robots.Count && i < multiTargets.Count; i++)
                {
                    var r = snapshot.Robots[i];
                    var t = multiTargets[i];
                    var position = Pose2D.Lerp(r.PreviousPosition, r.Position, interpolationAlpha);

                    float rx = ox + (position.X / map.CellSizeMeters) * cellPixels;
                    float ry = oy + (position.Y / map.CellSizeMeters) * cellPixels;
                    float tx = ox + (t.X + 0.5f) * cellPixels;
                    float ty = oy + (t.Y + 0.5f) * cellPixels;

                    canvas.DrawLine(rx, ry, tx, ty, linePaint);
                }
            }
        }

        private void DrawPlannedPaths(SKCanvas canvas, SimulationSnapshot snapshot, GridMap map, float ox, float oy, float cellPixels, float interpolationAlpha)
        {
            // 绘制机群各机器人的未来规划路径折线 (不绘制已走过的历史路段与向后倒拉线)。
            if (snapshot?.Robots == null) return;

            for (int i = 0; i < snapshot.Robots.Count; i++)
            {
                var r = snapshot.Robots[i];
                if (r.CurrentPath == null || r.CurrentPath.Count <= 1) continue;

                int targetIdx = Math.Clamp(r.CurrentTargetIndex, 0, r.CurrentPath.Count);
                if (targetIdx >= r.CurrentPath.Count) continue; // 全部路径已走完

                var position = Pose2D.Lerp(r.PreviousPosition, r.Position, interpolationAlpha);

                using (var pathLine = new SKPath())
                {
                    float startX = ox + (position.X / map.CellSizeMeters) * cellPixels;
                    float startY = oy + (position.Y / map.CellSizeMeters) * cellPixels;
                    pathLine.MoveTo(startX, startY);

                    for (int p = targetIdx; p < r.CurrentPath.Count; p++)
                    {
                        var pt = r.CurrentPath[p];
                        float px = ox + (pt.X + 0.5f) * cellPixels;
                        float py = oy + (pt.Y + 0.5f) * cellPixels;
                        pathLine.LineTo(px, py);
                    }

                    using (var pathPaint = new SKPaint
                    {
                        Color = UnifiedPathColor.WithAlpha(170),
                        Style = SKPaintStyle.Stroke,
                        StrokeWidth = Math.Max(2f, cellPixels * 0.08f),
                        PathEffect = SKPathEffect.CreateDash(new[] { 6f, 4f }, 0),
                        StrokeCap = SKStrokeCap.Round,
                        StrokeJoin = SKStrokeJoin.Round,
                        IsAntialias = true
                    })
                    {
                        canvas.DrawPath(pathLine, pathPaint);
                    }
                }
            }
        }

        private void DrawRobots(SKCanvas canvas, SimulationSnapshot snapshot, GridMap map, float ox, float oy, float cellPixels, float interpolationAlpha, int? selectedRobotId)
        {
            // 绘制机器人实体底盘、阴影、航向箭头与选中小车高亮光圈。
            if (snapshot?.Robots == null) return;

            float bodyLength = (0.42f / map.CellSizeMeters) * cellPixels;
            float bodyWidth = (0.38f / map.CellSizeMeters) * cellPixels;
            var bodyRect = new SKRect(-bodyLength * 0.5f, -bodyWidth * 0.5f, bodyLength * 0.5f, bodyWidth * 0.5f);
            if (bodyLength != _arrowBodyLength || bodyWidth != _arrowBodyWidth)
            {
                _arrowPath.Reset();
                float tipX = bodyLength * 0.42f;
                float backX = -bodyLength * 0.15f;
                float halfSpan = bodyWidth * 0.32f;
                _arrowPath.MoveTo(tipX, 0);
                _arrowPath.LineTo(backX, -halfSpan);
                _arrowPath.LineTo(backX * 0.5f, 0);
                _arrowPath.LineTo(backX, halfSpan);
                _arrowPath.Close();
                _arrowBodyLength = bodyLength;
                _arrowBodyWidth = bodyWidth;
            }

            for (int i = 0; i < snapshot.Robots.Count; i++)
            {
                var r = snapshot.Robots[i];
                var position = Pose2D.Lerp(r.PreviousPosition, r.Position, interpolationAlpha);
                _bodyPaint.Color = RobotWarmYellowColor;

                float centerPx = ox + (position.X / map.CellSizeMeters) * cellPixels;
                float centerPy = oy + (position.Y / map.CellSizeMeters) * cellPixels;

                canvas.Save();
                canvas.Translate(centerPx, centerPy);

                float angleDegrees = position.Heading * (180f / (float)Math.PI);
                canvas.RotateDegrees(angleDegrees);

                // 选中高亮光环 (随机器人局部朝向同步自旋)
                if (selectedRobotId.HasValue && r.Id == selectedRobotId.Value)
                {
                    float haloPad = Math.Max(2f, cellPixels * 0.08f);
                    var haloRect = new SKRect(bodyRect.Left - haloPad, bodyRect.Top - haloPad, bodyRect.Right + haloPad, bodyRect.Bottom + haloPad);
                    canvas.DrawRoundRect(haloRect, 6f, 6f, _selectionHaloPaint);
                }

                var shadowRect = new SKRect(bodyRect.Left + 2, bodyRect.Top + 2, bodyRect.Right + 2, bodyRect.Bottom + 2);
                canvas.DrawRoundRect(shadowRect, 4f, 4f, _shadowPaint);
                canvas.DrawRoundRect(bodyRect, 4f, 4f, _bodyPaint);
                canvas.DrawRoundRect(bodyRect, 4f, 4f, _bodyStrokePaint);
                canvas.DrawPath(_arrowPath, _arrowPaint);
                float centerRadius = Math.Max(2f, bodyWidth * 0.16f);
                canvas.DrawCircle(0f, 0f, centerRadius, _centerPaint);

                canvas.Restore();
            }
        }

        private void DrawMultiRobotBadges(SKCanvas canvas, SimulationSnapshot snapshot, GridMap map, float ox, float oy, float cellPixels, float interpolationAlpha)
        {
            // 绘制机器人右上方的编号徽标 (发生碰撞的机器人背景从深黑色变为醒目红色警示)。
            for (int i = 0; i < snapshot.Robots.Count; i++)
            {
                var r = snapshot.Robots[i];
                var position = Pose2D.Lerp(r.PreviousPosition, r.Position, interpolationAlpha);
                float centerPx = ox + (position.X / map.CellSizeMeters) * cellPixels;
                float centerPy = oy + (position.Y / map.CellSizeMeters) * cellPixels;

                float badgeX = centerPx + cellPixels * 0.35f;
                float badgeY = centerPy - cellPixels * 0.35f;
                var bgPaint = r.HasCollided ? _badgeCollisionPaint : _badgeBackgroundPaint;
                canvas.DrawCircle(badgeX, badgeY, 8f, bgPaint);
                canvas.DrawText($"{r.Id}", badgeX, badgeY + 3.5f, _badgeTextPaint);
            }
        }

        public void Dispose()
        {
            // 释放渲染器持有的 Skia 资源。
            _floorPaint.Dispose();
            _gridPaint.Dispose();
            _borderPaint.Dispose();
            _shadowPaint.Dispose();
            _bodyPaint.Dispose();
            _bodyStrokePaint.Dispose();
            _arrowPaint.Dispose();
            _centerPaint.Dispose();
            _badgeBackgroundPaint.Dispose();
            _badgeCollisionPaint.Dispose();
            _badgeTextPaint.Dispose();
            _selectionHaloPaint.Dispose();
            _arrowPath.Dispose();
        }

        public void DrawMinimizeButton(SKCanvas canvas, SKRect btnRect, bool isMinimized)
        {
            // 绘制 HUD 卡片右上角的折叠收起与展开切换按钮。
            using (var bgPaint = new SKPaint { Color = new SKColor(228, 235, 242, 220), Style = SKPaintStyle.Fill, IsAntialias = true })
            using (var borderPaint = new SKPaint { Color = new SKColor(180, 195, 210, 220), Style = SKPaintStyle.Stroke, StrokeWidth = 1f, IsAntialias = true })
            using (var iconPaint = new SKPaint
            {
                Color = new SKColor(60, 80, 105),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1.6f,
                StrokeCap = SKStrokeCap.Round,
                IsAntialias = true
            })
            {
                canvas.DrawRoundRect(btnRect, 3f, 3f, bgPaint);
                canvas.DrawRoundRect(btnRect, 3f, 3f, borderPaint);

                float cx = btnRect.MidX;
                float cy = btnRect.MidY;
                float halfW = 4f;

                canvas.DrawLine(cx - halfW, cy, cx + halfW, cy, iconPaint);
                if (isMinimized)
                {
                    canvas.DrawLine(cx, cy - halfW, cx, cy + halfW, iconPaint);
                }
            }
        }

        private void DrawViewportHud(
            SKCanvas canvas,
            GridMap map,
            SimulationMode mode,
            SimulationSnapshot snapshot,
            bool isMinimized,
            ref SKRect hudRect,
            ref SKRect minBtnRect)
        {
            // 在左上角使用 SkiaSharp 绘制圆角 HUD 视口参数说明卡片。
            float cardWidth = 320f;
            float cardHeight = isMinimized ? 32f : 116f;
            hudRect = new SKRect(12f, 12f, 12f + cardWidth, 12f + cardHeight);
            minBtnRect = new SKRect(hudRect.Right - 24f, hudRect.Top + 7f, hudRect.Right - 6f, hudRect.Top + 25f);

            using (var panelPaint = new SKPaint { Color = new SKColor(255, 255, 255, 235), Style = SKPaintStyle.Fill, IsAntialias = true })
            using (var strokePaint = new SKPaint { Color = new SKColor(190, 200, 210), Style = SKPaintStyle.Stroke, StrokeWidth = 1f, IsAntialias = true })
            using (var textPaint = new SKPaint { Color = new SKColor(40, 50, 60), TextSize = 12f, IsAntialias = true, Typeface = YaHeiTypeface })
            using (var titlePaint = new SKPaint { Color = new SKColor(25, 40, 55), TextSize = 12.5f, IsAntialias = true, Typeface = YaHeiBoldTypeface })
            {
                canvas.DrawRoundRect(hudRect, 6f, 6f, panelPaint);
                canvas.DrawRoundRect(hudRect, 6f, 6f, strokePaint);

                DrawMinimizeButton(canvas, minBtnRect, isMinimized);

                string modeDesc = (mode == SimulationMode.SingleRobot)
                    ? "单机模式"
                    : $"多机调度 ({snapshot?.Robots?.Count ?? 0}台)";

                canvas.DrawText($"{map.Width} x {map.Height} 仓储栅格地图 ({modeDesc})", 20f, 32f, titlePaint);

                if (isMinimized) return;

                canvas.DrawText($"单格尺寸: {map.CellSizeMeters:F2}m | 机器人离散中心对齐", 20f, 52f, textPaint);
                canvas.DrawText("视口操作: 鼠标左键拖拽平移 | 滚轮缩放", 20f, 72f, textPaint);

                if (mode == SimulationMode.SingleRobot)
                {
                    canvas.DrawText("目标设定: 单击网格可随时设定或更改目标", 20f, 92f, textPaint);
                    var robot = snapshot?.Robots?.FirstOrDefault();
                    string fsmDesc = robot?.MotionState switch
                    {
                        RobotMotionState.MovingForward => "自动移动中 (正在沿路径行进)",
                        RobotMotionState.TurningLeft or RobotMotionState.TurningRight => "原地自旋中 (航向对齐调整)",
                        _ => "闲置待命 (可改参数与点击寻路)"
                    };
                    canvas.DrawText($"状态机: {fsmDesc}", 20f, 112f, textPaint);
                }
                else
                {
                    canvas.DrawText("多机分布: 随机分布在独立网格，无碰撞重叠", 20f, 92f, textPaint);
                    canvas.DrawText("调度模式: 待命中 (右侧可随选数量与重分布)", 20f, 112f, textPaint);
                }
            }
        }

        private void DrawStatusMonitorHud(
            SKCanvas canvas,
            int width,
            int height,
            SimulationSnapshot snapshot,
            SimulationMode mode,
            System.Drawing.Point? selectedTarget,
            IReadOnlyList<System.Drawing.Point> multiTargets,
            bool isMinimized,
            ref SKRect hudRect,
            ref SKRect minBtnRect,
            float renderFps,
            int? selectedRobotId,
            double multiElapsedTime,
            int completedTasksCount,
            double taskEfficiency)
        {
            // 在左下角使用 SkiaSharp 绘制机群宏观整体状态或单机精细化状态监测数据表格。
            bool showSingleRobotDetail = (mode == SimulationMode.SingleRobot) || (selectedRobotId.HasValue);
            float cardWidth = 340f;
            int rowCount = showSingleRobotDetail ? 4 : 3;
            float rowH = 22f;
            float cardHeight = isMinimized ? 32f : (36f + rowCount * rowH + 8f);
            float cardX = 12f;
            float cardY = height - cardHeight - 12f;
            if (cardY < 135f) cardY = 135f;

            hudRect = new SKRect(cardX, cardY, cardX + cardWidth, cardY + cardHeight);
            minBtnRect = new SKRect(hudRect.Right - 24f, hudRect.Top + 6f, hudRect.Right - 6f, hudRect.Top + 24f);

            using (var panelPaint = new SKPaint { Color = new SKColor(255, 255, 255, 235), Style = SKPaintStyle.Fill, IsAntialias = true })
            using (var borderPaint = new SKPaint { Color = new SKColor(190, 200, 210), Style = SKPaintStyle.Stroke, StrokeWidth = 1f, IsAntialias = true })
            using (var titlePaint = new SKPaint { Color = new SKColor(25, 40, 55), TextSize = 12.5f, IsAntialias = true, Typeface = YaHeiBoldTypeface })
            using (var tipPaint = new SKPaint { Color = new SKColor(115, 130, 150), TextSize = 10f, IsAntialias = true, Typeface = YaHeiTypeface })
            using (var keyPaint = new SKPaint { Color = new SKColor(60, 75, 95), TextSize = 10.5f, IsAntialias = true, Typeface = YaHeiBoldTypeface })
            using (var valPaint = new SKPaint { Color = new SKColor(15, 23, 42), TextSize = 10.5f, IsAntialias = true, Typeface = YaHeiTypeface })
            using (var gridLinePaint = new SKPaint { Color = new SKColor(215, 222, 230, 220), Style = SKPaintStyle.Stroke, StrokeWidth = 1f })
            using (var zebraRowPaint = new SKPaint { Color = new SKColor(248, 250, 252, 140), Style = SKPaintStyle.Fill })
            {
                canvas.DrawRoundRect(hudRect, 6f, 6f, panelPaint);
                canvas.DrawRoundRect(hudRect, 6f, 6f, borderPaint);

                DrawMinimizeButton(canvas, minBtnRect, isMinimized);

                float tableX = cardX + 8f;
                float tableY = cardY + 27f;
                float tableW = cardWidth - 16f;
                float colKeyW = 62f;
                float colValW = (tableW - colKeyW * 2f) * 0.5f;

                string[,] rows;

                if (!showSingleRobotDetail)
                {
                    // 多机模式：未选中单车，显示整体状态
                    canvas.DrawText("多机集群整体状态监测", cardX + 10f, cardY + 20f, titlePaint);
                    if (isMinimized) return;

                    int robotCount = snapshot?.Robots?.Count ?? 0;
                    double displayTime = multiElapsedTime > 0.0 ? multiElapsedTime : (snapshot?.SimulationTime ?? 0f);
                    float avgWait = (snapshot?.Robots != null && snapshot.Robots.Count > 0)
                        ? snapshot.Robots.Average(r => r.WaitTimeSeconds)
                        : 0f;
                    int collisionCount = snapshot?.TotalCollisionCount ?? 0;

                    rows = new string[3, 4]
                    {
                        { "机器人数量", $"{robotCount} 台", "运行时间", $"{displayTime:F1} s" },
                        { "任务完成数", $"{completedTasksCount} 个", "任务效率", $"{taskEfficiency:F1} 个/分" },
                        { "平均等待时间", $"{avgWait:F1} s", "碰撞次数", $"{collisionCount} 次" }
                    };
                }
                else
                {
                    // 选中小车（或单机模式）：显示该机器人的精细化状态
                    RobotSnapshot robot = null;
                    if (selectedRobotId.HasValue && snapshot?.Robots != null)
                    {
                        robot = snapshot.Robots.FirstOrDefault(r => r.Id == selectedRobotId.Value);
                    }
                    robot ??= snapshot?.Robots?.FirstOrDefault();

                    if (mode == SimulationMode.MultiRobot)
                    {
                        canvas.DrawText($"机器人 #{robot?.Id ?? 0} 状态监测", cardX + 10f, cardY + 20f, titlePaint);
                        canvas.DrawText("(点击空白返回整体)", cardX + 130f, cardY + 20f, tipPaint);
                    }
                    else
                    {
                        canvas.DrawText($"机器人 #{robot?.Id ?? 0} 状态监测 [单机模式]", cardX + 10f, cardY + 20f, titlePaint);
                    }

                    if (isMinimized) return;

                    string idStr = robot != null ? $"#{robot.Id}" : "-";
                    string batteryStr = robot != null ? $"{robot.BatteryPercentage:F0}%" : "-";
                    string posStr = robot != null ? $"({robot.GridPosition.X}, {robot.GridPosition.Y})" : "-";
                    string speedStr = robot != null ? $"{robot.LinearVelocity:F2} m/s" : "-";

                    string headingStr = "-";
                    if (robot != null)
                    {
                        float headingDeg = (float)(((robot.Position.Heading * 180f / Math.PI) % 360 + 360) % 360);
                        string headingDir = headingDeg switch
                        {
                            >= 315 or < 45 => "东",
                            >= 45 and < 135 => "南",
                            >= 135 and < 225 => "西",
                            _ => "北"
                        };
                        headingStr = $"{headingDeg:F0}° ({headingDir})";
                    }

                    string targetStr = "-";
                    if (mode == SimulationMode.SingleRobot)
                    {
                        targetStr = selectedTarget.HasValue ? $"({selectedTarget.Value.X}, {selectedTarget.Value.Y})" : "未设置";
                    }
                    else if (robot != null && snapshot?.Robots != null)
                    {
                        int rIdx = -1;
                        for (int i = 0; i < snapshot.Robots.Count; i++)
                        {
                            if (snapshot.Robots[i].Id == robot.Id) { rIdx = i; break; }
                        }
                        if (rIdx >= 0 && rIdx < multiTargets.Count)
                        {
                            targetStr = $"({multiTargets[rIdx].X}, {multiTargets[rIdx].Y})";
                        }
                        else if (robot.TargetGrid.HasValue)
                        {
                            targetStr = $"({robot.TargetGrid.Value.X}, {robot.TargetGrid.Value.Y})";
                        }
                    }

                    string taskStatusStr;
                    if (robot == null)
                    {
                        taskStatusStr = "-";
                    }
                    else if (robot.IsAwaitingAuthorization)
                    {
                        taskStatusStr = robot.BlockingRobotId.HasValue
                            ? $"等待资源 (#{robot.BlockingRobotId.Value})"
                            : "等待资源";
                    }
                    else if (robot.MotionState == RobotMotionState.MovingForward ||
                             robot.MotionState == RobotMotionState.TurningLeft ||
                             robot.MotionState == RobotMotionState.TurningRight)
                    {
                        taskStatusStr = "任务执行中";
                    }
                    else
                    {
                        taskStatusStr = "空闲";
                    }

                    string cargoStr = robot != null ? (robot.HasCargo ? "负载" : "空载") : "-";

                    rows = new string[4, 4]
                    {
                        { "机器人ID", idStr, "电量", batteryStr },
                        { "位置", posStr, "速度", speedStr },
                        { "朝向", headingStr, "目标位置", targetStr },
                        { "任务状态", taskStatusStr, "载货状态", cargoStr }
                    };
                }

                for (int r = 0; r < rowCount; r++)
                {
                    float ry = tableY + r * rowH;
                    if (r % 2 == 1)
                    {
                        canvas.DrawRect(tableX, ry, tableW, rowH, zebraRowPaint);
                    }
                    canvas.DrawLine(tableX, ry, tableX + tableW, ry, gridLinePaint);

                    canvas.DrawText(rows[r, 0], tableX + 4f, ry + 15f, keyPaint);
                    canvas.DrawText(rows[r, 1], tableX + colKeyW + 4f, ry + 15f, valPaint);
                    canvas.DrawText(rows[r, 2], tableX + colKeyW + colValW + 4f, ry + 15f, keyPaint);
                    canvas.DrawText(rows[r, 3], tableX + colKeyW * 2 + colValW + 4f, ry + 15f, valPaint);
                }
                canvas.DrawRect(tableX, tableY, tableW, rowH * rowCount, gridLinePaint);
            }
        }
    }
}
