using System;
using System.Collections.Generic;
using SkiaSharp;

namespace WarehouseGridSimulation.Skia.UI
{
    /// <summary>
    /// Skia UI 交互组件管理器。
    /// 承载所有由 SkiaSharp 绘制的 UI 控件，接管窗口宿主转发的鼠标事件并执行命中测试。
    /// </summary>
    public class SkiaUiManager
    {
        private readonly List<SkiaUiButton> _buttons = new List<SkiaUiButton>();
        private SkiaUiButton _pressedButton = null;

        public IReadOnlyList<SkiaUiButton> Buttons => _buttons;

        public void Clear()
        {
            _buttons.Clear();
            _pressedButton = null;
        }

        public void AddButton(SkiaUiButton button)
        {
            if (button != null) _buttons.Add(button);
        }

        public bool OnMouseDown(float x, float y)
        {
            foreach (var btn in _buttons)
            {
                if (btn.Contains(x, y))
                {
                    btn.IsPressed = true;
                    _pressedButton = btn;
                    return true;
                }
            }
            return false;
        }

        public bool OnMouseMove(float x, float y)
        {
            bool handled = false;
            foreach (var btn in _buttons)
            {
                bool inside = btn.Contains(x, y);
                if (inside) handled = true;
                btn.IsHovered = inside;
            }
            return handled;
        }

        public bool OnMouseUp(float x, float y)
        {
            bool clicked = false;
            if (_pressedButton != null)
            {
                if (_pressedButton.Contains(x, y))
                {
                    _pressedButton.TriggerClick();
                    clicked = true;
                }
                _pressedButton.IsPressed = false;
                _pressedButton = null;
            }

            foreach (var btn in _buttons)
            {
                btn.IsPressed = false;
            }

            return clicked;
        }

        public void Draw(SKCanvas canvas, SKTypeface typeface)
        {
            foreach (var btn in _buttons)
            {
                btn.Draw(canvas, typeface);
            }
        }
    }
}
