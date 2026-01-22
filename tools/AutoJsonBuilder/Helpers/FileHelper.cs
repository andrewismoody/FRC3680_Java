using System;
using System.IO;
using System.Linq;

namespace AutoJsonBuilder.Helpers;

internal static class FileHelper
{
	public static string? FindSolutionRoot()
	{
		var dir = new DirectoryInfo(Path.GetFullPath(AppContext.BaseDirectory));
		while (dir != null)
		{
			if (File.Exists(Path.Combine(dir.FullName, "FRC3680-Java_2025.sln")))
				return dir.FullName;
			dir = dir.Parent;
		}
		return null;
	}

	public static string? GetDefaultDeployFolder()
	{
		var root = FindSolutionRoot();
		if (root is null) return null;

		try
		{
			var deploy = Directory.EnumerateDirectories(root, "deploy", SearchOption.AllDirectories)
								  .FirstOrDefault();
			if (!string.IsNullOrWhiteSpace(deploy)) return Path.GetFullPath(deploy);

			var candidate = Path.Combine(root, "tools", "AutoJsonBuilder", "deploy");
			if (Directory.Exists(candidate)) return candidate;
		}
		catch
		{
			// ignore IO errors
		}
		return null;
	}
}
