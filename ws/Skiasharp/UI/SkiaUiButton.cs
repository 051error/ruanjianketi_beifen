using System;
using SkiaSharp;

namespace WarehouseGridSimulation.Skia.UI
{
    /// <summary>
    /// 由 SkiaSharp 直接在画布上绘制的矢量交互按钮。
    /// 不依赖 WinForms 原生控件，支持高分屏平滑抗锯齿、鼠标悬停与点击状态。
    /// </summary>
    public class SkiaUiButton
    {
        public SKRect Bounds { get; set; }
        public string Text { get; set; }
        public SKColor NormalColor { get; set; } = new SKColor(45, 120, 215);
        public SKColor HoverColor { get; set; } = new SKColor(65, 145, 240);
        public SKColor PressedColor { get; set; } = new SKColor(30, 95, 180);
        public SKColor TextColor { get; set; } = SKColors.White;
        public float CornerRadius { get; set; } = 6f;
        public float FontSize { get; set; } = 13f;

        public bool IsHovered { get; internal set; }
        public bool IsPressed { get; internal set; }

        public event Action Clicked;

        public SkiaUiButton(string text, SKRect bounds, Action onClick = null)
        {
            Text = text;
            Bounds = bounds;
            if (onClick != null) Clicked += onClick;
        }

        public bool Contains(float x, float y)
        {
            return x >= Bounds.Left - 2 && x <= Bounds.Right + 2 &&
                   y >= Bounds.Top - 2 && y <= Bounds.Bottom + 2;
        }

        public void TriggerClick() => Clicked?.Invoke();

        public void Draw(SKCanvas canvas, SKTypeface typeface)
        {
            SKColor currentBg = NormalColor;
            if (IsPressed) currentBg = PressedColor;
            else if (IsHovered) currentBg = HoverColor;

            // 1. 绘制按钮阴影与圆角底色
            using (var shadowPaint = new SKPaint
            {
                Color = new SKColor(0, 0, 0, 30),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            {
                var shadowRect = new SKRoundRect(new SKRect(Bounds.Left, Bounds.Top + 2, Bounds.Right, Bounds.Bottom + 2), CornerRadius);
                canvas.DrawRoundRect(shadowRect, shadowPaint);
            }

            using (var bgPaint = new SKPaint
            {
                Color = currentBg,
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            })
            {
                var btnRect = new SKRoundRect(Bounds, CornerRadius);
                canvas.DrawRoundRect(btnRect, bgPaint);
            }

            // 2. 绘制微弱外边框
            using (var borderPaint = new SKPaint
            {
                Color = new SKColor(255, 255, 255, 60),
                Style = SKPaintStyle.Stroke,
                StrokeWidth = 1f,
                IsAntialias = true
            })
            {
                var btnRect = new SKRoundRect(Bounds, CornerRadius);
                canvas.DrawRoundRect(btnRect, borderPaint);
            }

            // 3. 居中绘制按钮文本
            using (var textPaint = new SKPaint
            {
                Color = TextColor,
                Typeface = typeface,
                TextSize = FontSize,
                IsAntialias = true,
                TextAlign = SKTextAlign.Center
            })
            {
                var metrics = textPaint.FontMetrics;
                float textY = Bounds.MidY - (metrics.Ascent + metrics.Descent) * 0.5f;
                canvas.DrawText(Text, Bounds.MidX, textY, textPaint);
            }
        }
    }
}
