using System;
using System.Drawing;
using System.Windows.Forms;
using SkiaSharp;

namespace WarehouseGridSimulation.Skia.UI
{
    /// <summary>
    /// 视口与二维相机控制器。
    /// 统一管理世界地图在画布中的平移、缩放、网格居中、坐标双向投影及 HUD 区域过滤。
    /// </summary>
    public class ViewportController
    {
        public const float BaseCellPixels = 24f;
        public const float MinimumScale = 0.35f;
        public const float MaximumScale = 3.5f;

        public float Scale { get; set; } = 1.0f;
        public float OffsetX { get; set; } = 40f;
        public float OffsetY { get; set; } = 80f;

        public bool IsPanning { get; private set; }
        private Point _lastMousePosition;
        private Point _mouseDownLocation;

        public SKRect ViewportHudRect;
        public SKRect StatusHudRect;
        public SKRect ViewportMinBtnRect;
        public SKRect StatusMinBtnRect;
        public bool IsViewportHudMinimized { get; set; }
        public bool IsStatusHudMinimized { get; set; }

        public float CellPixels => BaseCellPixels * Scale;

        public void CenterGridInView(int viewWidth, int viewHeight, int mapWidth, int mapHeight)
        {
            // 将指定尺寸的网格地图平移居中于可视窗口。
            // viewWidth: 可视区域宽度; viewHeight: 可视区域高度; mapWidth: 地图横向网格数; mapHeight: 地图纵向网格数。
            float gridWidth = mapWidth * CellPixels;
            float gridHeight = mapHeight * CellPixels;

            OffsetX = (viewWidth - gridWidth) * 0.5f;
            OffsetY = Math.Max(65f, (viewHeight - gridHeight) * 0.5f);
        }

        public bool HandleMouseDown(Point location, MouseButtons button, out bool hudMinBtnClicked)
        {
            // 响应鼠标按下事件并执行 HUD 折叠按钮检测或启动平移。
            // location: 鼠标事件发生位置; button: 按下的鼠标按键; hudMinBtnClicked: 是否命中了折叠展开按钮。
            hudMinBtnClicked = false;
            _mouseDownLocation = location;

            if (ViewportMinBtnRect.Contains(location.X, location.Y))
            {
                IsViewportHudMinimized = !IsViewportHudMinimized;
                hudMinBtnClicked = true;
                return true;
            }
            if (StatusMinBtnRect.Contains(location.X, location.Y))
            {
                IsStatusHudMinimized = !IsStatusHudMinimized;
                hudMinBtnClicked = true;
                return true;
            }

            if (ViewportHudRect.Contains(location.X, location.Y) || StatusHudRect.Contains(location.X, location.Y))
            {
                return true;
            }

            if (button == MouseButtons.Left || button == MouseButtons.Middle)
            {
                IsPanning = true;
                _lastMousePosition = location;
            }

            return false;
        }

        public bool HandleMouseMove(Point location)
        {
            // 响应鼠标移动事件并更新视口平移偏移。
            // location: 鼠标移动后的当前位置坐标。
            if (IsPanning)
            {
                float dx = location.X - _lastMousePosition.X;
                float dy = location.Y - _lastMousePosition.Y;
                OffsetX += dx;
                OffsetY += dy;
                _lastMousePosition = location;
                return true;
            }
            return false;
        }

        public bool HandleMouseUp(Point location, out bool isClick)
        {
            // 响应鼠标抬起事件并判断是否属于未拖拽的单纯点击。
            // isClick: 是否判定为网格拾取单击操作。
            int dragDistX = Math.Abs(location.X - _mouseDownLocation.X);
            int dragDistY = Math.Abs(location.Y - _mouseDownLocation.Y);
            isClick = dragDistX < 4 && dragDistY < 4;

            if (IsPanning)
            {
                IsPanning = false;
                return true;
            }
            return false;
        }

        public bool HandleMouseWheel(Point mousePos, int delta)
        {
            // 响应滚轮滚动并以当前鼠标光标所在像素位置为锚点缩放视口。
            // mousePos: 鼠标指针当前屏幕像素坐标; delta: 滚轮滚动刻度增量。
            float zoomFactor = delta > 0 ? 1.15f : 0.87f;
            float newScale = Math.Clamp(Scale * zoomFactor, MinimumScale, MaximumScale);

            if (Math.Abs(newScale - Scale) > 0.001f)
            {
                OffsetX = mousePos.X - (mousePos.X - OffsetX) * (newScale / Scale);
                OffsetY = mousePos.Y - (mousePos.Y - OffsetY) * (newScale / Scale);
                Scale = newScale;
                return true;
            }
            return false;
        }

        public bool ScreenToGrid(Point screenPoint, int mapWidth, int mapHeight, out Point gridPoint)
        {
            // 将屏幕鼠标像素坐标逆变换为地图内部离散栅格坐标。
            // screenPoint: 屏幕鼠标坐标; mapWidth: 地图网格总宽度; mapHeight: 地图网格总高度; gridPoint: 转换后的离散网格坐标。
            gridPoint = Point.Empty;
            if (ViewportHudRect.Contains(screenPoint.X, screenPoint.Y) || StatusHudRect.Contains(screenPoint.X, screenPoint.Y))
            {
                return false;
            }

            float currentCellPixels = CellPixels;
            int gx = (int)Math.Floor((screenPoint.X - OffsetX) / currentCellPixels);
            int gy = (int)Math.Floor((screenPoint.Y - OffsetY) / currentCellPixels);

            if (gx >= 0 && gx < mapWidth && gy >= 0 && gy < mapHeight)
            {
                gridPoint = new Point(gx, gy);
                return true;
            }
            return false;
        }
    }
}
