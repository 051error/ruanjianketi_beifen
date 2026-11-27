using System;
using System.Windows.Forms;
using WarehouseGridSimulation.Skia.Forms;

namespace WarehouseGridSimulation.Skia
{
    /// <summary>
    /// 包含 SkiaSharp 仓储多智能体仿真程序的入口点。
    /// </summary>
    internal static class Program
    {
        [STAThread]
        private static void Main()
        {
            // 初始化 Windows 控件视觉样式并启动主仿真窗体
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.SetUnhandledExceptionMode(UnhandledExceptionMode.CatchException);
            Application.ThreadException += (s, e) =>
            {
                System.Diagnostics.Debug.WriteLine($"[UI线程异常捕获] {e.Exception.Message}");
            };
            AppDomain.CurrentDomain.UnhandledException += (s, e) =>
            {
                System.Diagnostics.Debug.WriteLine($"[全局未处理异常] {e.ExceptionObject}");
            };
            Application.Run(new MainForm());
        }
    }
}
