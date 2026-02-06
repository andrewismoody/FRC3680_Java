using System;
using System.Runtime.InteropServices;
using System.Windows;

namespace AutoJsonBuilder
{
	// Single entrypoint: CLI when args present, GUI when none.
	public static class Program
	{
		// When project is WinExe there is no console; AllocConsole makes one so CLI output is visible.
		[DllImport("kernel32.dll", SetLastError = true)]
		private static extern bool AllocConsole();

		[STAThread]
		public static int Main(string[] args)
		{
			try
			{
				if (args != null && args.Length > 0)
				{
					// Create console if needed (no-op when already attached)
					try { AllocConsole(); } catch { /* best-effort */ }

					// Call the CLI entry you already added (returns exit code)
					return AutoJsonBuilder.Tools.RandomAutoJsonCli.Main(args);
				}
				else
				{
					// GUI mode: start WPF App as usual
					var app = new App();
					app.InitializeComponent();
					app.Run();
					return 0;
				}
			}
			catch (Exception ex)
			{
				// Minimal fallback logging for both CLI and GUI starts
				try { Console.Error.WriteLine($"Startup error: {ex.Message}"); } catch { }
				return 1;
			}
		}
	}
}
