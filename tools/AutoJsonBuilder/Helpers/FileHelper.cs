using System;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using Microsoft.Win32;
using AutoJsonBuilder.Models;

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

    // Prompt user to select an auto JSON file to open. Returns full path or null if cancelled.
    public static string? PromptOpenAutoJsonPath()
    {
        var dlg = new OpenFileDialog
        {
            Title = "Open auto JSON",
            Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*",
            CheckFileExists = true
        };
        return dlg.ShowDialog() == true ? dlg.FileName : null;
    }

    // Prompt user to choose a path to save an auto JSON. Returns full path or null if cancelled.
    public static string? PromptSaveAutoJsonPath(string suggestedFileName)
    {
        var dlg = new SaveFileDialog
        {
            Title = "Save auto JSON",
            Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*",
            FileName = suggestedFileName ?? "auto.json",
            OverwritePrompt = true,
            InitialDirectory = GetDefaultDeployFolder() ?? Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments)
        };
        return dlg.ShowDialog() == true ? dlg.FileName : null;
    }

    // Write the provided AutoDefinitionModel to the given path using the app's canonical camel-case indented serializer.
    public static Task WriteAutoJsonAsync(string path, AutoDefinitionModel doc)
    {
        var json = JsonHelper.SerializeCamelIndented(doc);
        return File.WriteAllTextAsync(path, json);
    }

    // Simple wrapper to read all text (keeps calling code concise)
    public static Task<string> ReadAllTextAsync(string path) => File.ReadAllTextAsync(path);
}
