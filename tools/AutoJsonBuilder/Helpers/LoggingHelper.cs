
using System;
using System.Collections.Generic;
using AutoJsonBuilder;

namespace AutoJsonBuilder.Helpers;

internal static class LoggingHelper
{
	public static void LogAllNodeBindings(IEnumerable<TreeItemVm> roots, Action<string> logger)
	{
		if (roots is null || logger is null) return;
		foreach (var root in roots)
			LogNodeBindings(root, logger);
	}

	private static void LogNodeBindings(TreeItemVm node, Action<string> logger, string path = "")
	{
		var myPath = string.IsNullOrEmpty(path) ? node.Title : $"{path}/{node.Title}";
		var val = node.Value is null ? "(null)" : node.Value.ToString() ?? "(null str)";
		var opts = node.Options?.Count ?? 0;
		var editor = node.Editor;

		logger($"NodeBinding: Path='{myPath}' Editor={editor} Value='{val}' Options={opts}");

		foreach (var c in node.Children)
			LogNodeBindings(c, logger, myPath);
	}
}