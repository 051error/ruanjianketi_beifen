using System;
using System.Windows.Forms;

namespace WarehouseGridSimulation
{
    /// <summary>
    /// Contains the Windows Forms application entry point.
    /// </summary>
    internal static class Program
    {
        /// <summary>
        /// Starts the warehouse grid simulation.
        /// </summary>
        [STAThread]
        private static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MainForm());
        }
    }
}
