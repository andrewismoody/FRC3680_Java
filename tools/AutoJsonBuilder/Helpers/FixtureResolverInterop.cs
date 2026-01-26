using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace AutoJsonBuilder.Helpers;

internal static class FixtureResolverInterop
{
	// cached java executable path
	private static string? _cachedJavaExe;

	// Resolve: sends JSON doc (single-line) and returns single-line JSON response or null on failure.
	public static string? Resolve(string requestJson, int timeoutMs = 5000)
	{
		if (string.IsNullOrWhiteSpace(requestJson)) return null;

		try
		{
			// Write request JSON to a temp .json file and invoke ResolvedFixtureDump once.
			var tmpPath = Path.Combine(Path.GetTempPath(), "autoresolve_" + Guid.NewGuid().ToString("N") + ".json");
			// Use UTF-8 without BOM so the Java/Jackson parser doesn't see a stray BOM character.
			File.WriteAllText(tmpPath, requestJson, new UTF8Encoding(encoderShouldEmitUTF8Identifier: false));

			try
			{
				var javaExe = GetJavaExecutable();
				if (string.IsNullOrWhiteSpace(javaExe)) return null;

				var classpath = FindFixtureResolverClasspath();
				if (string.IsNullOrWhiteSpace(classpath)) return null;

				var psi = new ProcessStartInfo
				{
					FileName = javaExe,
					Arguments = $"-cp \"{classpath}\" frc.robot.auto.ResolvedFixtureDump --file \"{tmpPath}\"",
					UseShellExecute = false,
					RedirectStandardOutput = true,
					RedirectStandardError = true,
					CreateNoWindow = true,
					StandardOutputEncoding = Encoding.UTF8,
					StandardErrorEncoding = Encoding.UTF8
				};

				using var proc = Process.Start(psi);
				if (proc == null) return null;

				var outTask = proc.StandardOutput.ReadToEndAsync();
				var errTask = proc.StandardError.ReadToEndAsync();

				if (proc.WaitForExit(timeoutMs))
				{
					var stdout = outTask.Result;
					var stderr = errTask.Result;

					if (!string.IsNullOrWhiteSpace(stderr))
					{
						Debug.WriteLine($"FixtureResolver stderr: {stderr.Trim()}");
					}

					return string.IsNullOrWhiteSpace(stdout) ? null : stdout.Trim();
				}
				else
				{
					// timeout
					try { proc.Kill(true); } catch { }
					return null;
				}
			}
			finally
			{
				try { File.Delete(tmpPath); } catch { }
			}
		}
		catch
		{
			return null;
		}
	}

	// Resolve by providing an existing on-disk JSON file path to ResolvedFixtureDump.
	public static string? ResolveFromFile(string jsonFilePath, int timeoutMs = 5000)
	{
		if (string.IsNullOrWhiteSpace(jsonFilePath) || !File.Exists(jsonFilePath)) return null;

		try
		{
			var javaExe = GetJavaExecutable();
			if (string.IsNullOrWhiteSpace(javaExe)) return null;

			var classpath = FindFixtureResolverClasspath();
			if (string.IsNullOrWhiteSpace(classpath)) return null;

			var psi = new ProcessStartInfo
			{
				FileName = javaExe,
				Arguments = $"-cp \"{classpath}\" frc.robot.auto.ResolvedFixtureDump --file \"{jsonFilePath}\"",
				UseShellExecute = false,
				RedirectStandardOutput = true,
				RedirectStandardError = true,
				CreateNoWindow = true,
				StandardOutputEncoding = Encoding.UTF8,
				StandardErrorEncoding = Encoding.UTF8
			};

			using var proc = Process.Start(psi);
			if (proc == null) return null;

			var outTask = proc.StandardOutput.ReadToEndAsync();
			var errTask = proc.StandardError.ReadToEndAsync();

			if (proc.WaitForExit(timeoutMs))
			{
				var stdout = outTask.Result;
				var stderr = errTask.Result;
				if (!string.IsNullOrWhiteSpace(stderr)) Debug.WriteLine($"FixtureResolver stderr: {stderr.Trim()}");
				return string.IsNullOrWhiteSpace(stdout) ? null : stdout.Trim();
			}
			else
			{
				try { proc.Kill(true); } catch { }
				return null;
			}
		}
		catch
		{
			return null;
		}
	}

	// No persistent server support — always invoke ResolvedFixtureDump per request (no fallback).

	// Try to find classpath using same project layout heuristics as AutoExprInterop
	private static string? FindFixtureResolverClasspath()
	{
		try
		{
			var root = FileHelper.FindSolutionRoot();
			if (string.IsNullOrWhiteSpace(root)) return null;

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
				var deployDir = Path.Combine(root, "tools", "AutoJsonBuilder", "deploy");
				if (Directory.Exists(deployDir))
					foreach (var jar in Directory.EnumerateFiles(deployDir, "*.jar"))
						entries.Add(jar);
				return string.Join(Path.PathSeparator.ToString(), entries);
			}

			var candidates = Directory.EnumerateDirectories(root, "classes", SearchOption.AllDirectories)
				.Where(d => d.Replace(Path.DirectorySeparatorChar, '/').Contains("/build/classes/java/main"))
				.ToList();
			if (candidates.Count > 0)
			{
				var chosenClasses = candidates[0];
				var chosenRoot = Path.GetFullPath(Path.Combine(chosenClasses, "..", "..", ".."));
				var chosenLibs = Path.Combine(chosenRoot, "libs");
				var entries = new List<string> { chosenClasses };
				if (Directory.Exists(chosenLibs))
					foreach (var jar in Directory.EnumerateFiles(chosenLibs, "*.jar"))
						entries.Add(jar);
				return string.Join(Path.PathSeparator.ToString(), entries);
			}
		}
		catch { }
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

	// optional: call on app shutdown (no-op, no persistent process)
	public static void Shutdown()
	{
		// intentionally empty — no persistent helper process to stop
	}
}
