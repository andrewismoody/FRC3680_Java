using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace AutoJsonBuilder.Helpers;

internal static class AutoExprInterop
{
	// persistent process members
	private static readonly object _sync = new();
	private static Process? _proc;
	private static StreamWriter? _procIn;
	private static StreamReader? _procOut;
	private static Task? _stderrReader;
	private static string? _cachedJavaExe;

	// Try to evaluate 'expr' using a persistent Java autoexpr.jar server.
	// paramsMap may be null or empty. Returns (success, value).
	public static (bool ok, double value) TryEvaluate(string expr, IDictionary<string,double>? paramsMap)
	{
		if (string.IsNullOrWhiteSpace(expr)) return (false, 0d);

		try
		{
			if (!EnsureStarted()) return (false, 0d);

			var req = new Dictionary<string, object?>
			{
				["expr"] = expr,
				["params"] = paramsMap ?? new Dictionary<string,double>()
			};
			var json = JsonSerializer.Serialize(req);

			lock (_sync)
			{
				_procIn!.WriteLine(json);
				_procIn.Flush();
			}

			// read single-line response with timeout
			var readTask = _procOut!.ReadLineAsync();
			if (readTask.Wait(TimeSpan.FromSeconds(2)))
			{
				var line = readTask.Result?.Trim();
				if (string.IsNullOrWhiteSpace(line)) return (false, 0d);

				// try plain number
				if (double.TryParse(line, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var plain))
					return (true, plain);

				// try JSON { "value": ... }
				try
				{
					using var doc = JsonDocument.Parse(line);
					if (doc.RootElement.ValueKind == JsonValueKind.Object && doc.RootElement.TryGetProperty("value", out var vEl))
					{
						if (vEl.ValueKind == JsonValueKind.Number && vEl.TryGetDouble(out var vd)) return (true, vd);
						if (vEl.ValueKind == JsonValueKind.String)
						{
							var s = vEl.GetString();
							if (double.TryParse(s, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var vs)) return (true, vs);
						}
					}
				}
				catch
				{
					// parse error -> treat as failure
				}
			}
			else
			{
				// timeout - restart process
				RestartProcess();
			}
		}
		catch
		{
			RestartProcess();
		}
		return (false, 0d);
	}

	// Ensure the persistent server is started
	private static bool EnsureStarted()
	{
		lock (_sync)
		{
			if (_proc != null && !_proc.HasExited && _procIn != null && _procOut != null) return true;

			RestartProcess();
			return _proc != null && !_proc.HasExited && _procIn != null && _procOut != null;
		}
	}

	private static void RestartProcess()
	{
		lock (_sync)
		{
			try
			{
				TryShutdown();

				// Prefer running the CLI class from the Gradle build output (same pattern as python interop).
					// Do not fall back to running an assembled jar.
				var javaExe = GetJavaExecutable();
				if (string.IsNullOrWhiteSpace(javaExe)) return;

				var classpath = FindAutoExprClasspath();
				if (string.IsNullOrWhiteSpace(classpath))
				{
					// no classpath available; do not attempt jar fallback per request
					return;
				}

				// launch: java -cp "<classpath>" frc.robot.auto.AutoExprCli --server
				var psi = new ProcessStartInfo
				{
					FileName = javaExe,
					Arguments = $"-cp \"{classpath}\" frc.robot.auto.AutoExprCli --server",
					UseShellExecute = false,
					RedirectStandardInput = true,
					RedirectStandardOutput = true,
					RedirectStandardError = true,
					CreateNoWindow = true,
				};

				_proc = Process.Start(psi);
				if (_proc == null) return;

				_procIn = _proc.StandardInput;
				_procOut = _proc.StandardOutput;

				// start background stderr reader to avoid blocking; ignore content but could log
				_stderrReader = Task.Run(async () =>
				{
					try
					{
						var sr = _proc.StandardError;
						string? s;
						while ((s = await sr.ReadLineAsync().ConfigureAwait(false)) != null)
						{
							// optionally route to UI log; omitted here to avoid dependency
						}
					}
					catch { }
				});
			}
			catch
			{
				TryShutdown();
			}
		}
	}

	private static void TryShutdown()
	{
		try
		{
			if (_procIn != null) { try { _procIn.Close(); } catch { } _procIn = null; }
			if (_procOut != null) { try { _procOut.Close(); } catch { } _procOut = null; }
			if (_proc != null)
			{
				try
				{
					if (!_proc.HasExited) _proc.Kill();
				}
				catch { }
				_proc.Dispose();
				_proc = null;
			}
		}
		catch { }
	}

	// find jar same as before
	// (Removed FindAutoExprJar — interop no longer uses a bundled jar)

	// Try to find Gradle build output classpath (classes dir + libs jars), matching the python interop pattern.
	// Returns a single classpath string (entries separated by Path.PathSeparator) or null if not found.
	private static string? FindAutoExprClasspath()
	{
		try
		{
			var root = FileHelper.FindSolutionRoot();
			if (string.IsNullOrWhiteSpace(root)) return null;

			// candidate: build/classes/java/main + build/libs/*.jar
			var classesDir = Path.Combine(root, "build", "classes", "java", "main");
			var libsDir = Path.Combine(root, "build", "libs");
			if (Directory.Exists(classesDir))
			{
				var entries = new List<string> { classesDir };
				if (Directory.Exists(libsDir))
				{
					foreach (var jar in Directory.EnumerateFiles(libsDir, "*.jar"))
						entries.Add(jar);
				}
				// also include any deploy jars if present
				var deployDir = Path.Combine(root, "tools", "AutoJsonBuilder", "deploy");
				if (Directory.Exists(deployDir))
					foreach (var jar in Directory.EnumerateFiles(deployDir, "*.jar"))
						entries.Add(jar);
				return string.Join(Path.PathSeparator.ToString(), entries);
			}

			// fallback: some setups put classes under module subfolder (e.g. AutoExpr/build/classes/...)
			// try scanning for any build/classes/java/main under tools or subprojects
			var candidates = Directory.EnumerateDirectories(root, "classes", SearchOption.AllDirectories)
				.Where(d => d.Replace(Path.DirectorySeparatorChar, '/').Contains("/build/classes/java/main"))
				.ToList();
			if (candidates.Count > 0)
			{
				// pick the first and include nearby libs
				var chosenClasses = candidates[0];
				var chosenRoot = Path.GetFullPath(Path.Combine(chosenClasses, "..", "..", "..")); // up to build
				var chosenLibs = Path.Combine(chosenRoot, "libs");
				var entries = new List<string> { chosenClasses };
				if (Directory.Exists(chosenLibs))
					foreach (var jar in Directory.EnumerateFiles(chosenLibs, "*.jar"))
						entries.Add(jar);
				return string.Join(Path.PathSeparator.ToString(), entries);
			}
		}
		catch { /* ignore */ }
		return null;
	}

	private static string? GetJavaExecutable()
	{
		if (_cachedJavaExe != null) return _cachedJavaExe;

		try
		{
			var javaHome = Environment.GetEnvironmentVariable("JAVA_HOME");
			if (!string.IsNullOrWhiteSpace(javaHome))
			{
				var candidate = Path.Combine(javaHome, "bin", Environment.OSVersion.Platform == PlatformID.Win32NT ? "java.exe" : "java");
				if (File.Exists(candidate))
				{
					_cachedJavaExe = candidate;
					return _cachedJavaExe;
				}
			}
		}
		catch { }

		try
		{
			var psi = new ProcessStartInfo
			{
				FileName = "java",
				Arguments = "-version",
				UseShellExecute = false,
				RedirectStandardError = true,
				RedirectStandardOutput = true,
				CreateNoWindow = true
			};
			using var p = Process.Start(psi);
			if (p != null)
			{
				if (p.WaitForExit(1500))
				{
					_cachedJavaExe = "java";
					return _cachedJavaExe;
				}
				try { p.Kill(); } catch { }
			}
		}
		catch { }

		return null;
	}

	// optional: call on app shutdown
	public static void Shutdown()
	{
		lock (_sync) TryShutdown();
	}
}
