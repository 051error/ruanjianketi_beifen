using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Forms;
using SkiaSharp;

namespace WarehouseGridSimulation.Skia.Controls
{
    /// <summary>
    /// 基于 SkiaSharp 打造的高性能 WinForms 渲染画布控件。
    /// </summary>
    public class SkiaCanvasControl : Control
    {
        private SKSurface _skSurface;
        private SKImageInfo _skImageInfo;
        private Bitmap _gdiBackBuffer;

        /// <summary>
        /// 当控件需要重绘 Skia 表面时触发。
        /// </summary>
        public event Action<SKCanvas, int, int> PaintSurface;

        public SkiaCanvasControl()
        {
            // 使用自有位图缓冲并避免重复的 WinForms 离屏缓冲
            SetStyle(ControlStyles.AllPaintingInWmPaint |
                     ControlStyles.UserPaint |
                     ControlStyles.ResizeRedraw, true);
            UpdateStyles();
        }

        protected override void OnPaintBackground(PaintEventArgs e)
        {
            // 使用完整画布绘制覆盖背景，避免重复擦除。
        }

        protected override void OnResize(EventArgs e)
        {
            // 响应控件尺寸变化并重建 Skia 渲染表面
            base.OnResize(e);
            RecreateBuffers();
            Invalidate();
        }

        private void RecreateBuffers()
        {
            // 释放旧缓冲并根据当前控件尺寸重新分配 Skia 渲染表面与 GDI 交换位图
            if (Width <= 0 || Height <= 0)
            {
                return;
            }

            // 核心优化：若当前缓冲区尺寸与控件当前宽高一致，避免反复销毁与非托管内存再分配
            if (_skSurface != null && _gdiBackBuffer != null &&
                _skImageInfo.Width == Width && _skImageInfo.Height == Height)
            {
                return;
            }

            if (_skSurface != null)
            {
                _skSurface.Dispose();
                _skSurface = null;
            }
            if (_gdiBackBuffer != null)
            {
                _gdiBackBuffer.Dispose();
                _gdiBackBuffer = null;
            }

            _skImageInfo = new SKImageInfo(Width, Height, SKColorType.Bgra8888, SKAlphaType.Premul);
            _skSurface = SKSurface.Create(_skImageInfo);
            _gdiBackBuffer = new Bitmap(Width, Height, PixelFormat.Format32bppPArgb);
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            // 将 SkiaSharp 绘制指令执行于表面并将像素快速传递到 GDI 窗口设备
            // e: 绘图事件参数，包含目标 GDI 绘图表面
            base.OnPaint(e);

            if (_skSurface == null || _gdiBackBuffer == null)
            {
                RecreateBuffers();
                if (_skSurface == null || _gdiBackBuffer == null)
                {
                    return;
                }
            }

            SKCanvas canvas = _skSurface.Canvas;
            if (PaintSurface != null)
            {
                PaintSurface(canvas, Width, Height);
            }

            BitmapData data = _gdiBackBuffer.LockBits(
                new Rectangle(0, 0, Width, Height),
                ImageLockMode.WriteOnly,
                PixelFormat.Format32bppPArgb);

            try
            {
                if (!_skSurface.ReadPixels(_skImageInfo, data.Scan0, data.Stride, 0, 0))
                {
                    throw new InvalidOperationException("无法读取 Skia 画布像素。");
                }
            }
            finally
            {
                _gdiBackBuffer.UnlockBits(data);
            }

            e.Graphics.DrawImageUnscaled(_gdiBackBuffer, 0, 0);
        }

        protected override void Dispose(bool disposing)
        {
            // 释放 SkiaSurface 与 GDI 位图占用的非托管及托管资源
            // disposing: 是否正在显式释放托管资源
            if (disposing)
            {
                if (_skSurface != null)
                {
                    _skSurface.Dispose();
                    _skSurface = null;
                }
                if (_gdiBackBuffer != null)
                {
                    _gdiBackBuffer.Dispose();
                    _gdiBackBuffer = null;
                }
            }
            base.Dispose(disposing);
        }
    }
}
