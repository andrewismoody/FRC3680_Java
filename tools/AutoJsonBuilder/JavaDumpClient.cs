using System.Diagnostics;
using System.IO;

namespace AutoJsonBuilder;

public sealed class JavaDumpClient
{
    public string JavaCmd { get; set; } = "java";
    public string Classpath { get; set; } = "";
    public string SeasonFile { get; set; } = "";

    // Keep WorkingDirectory if your classpath uses relative paths
    public string WorkingDirectory { get; set; } =
        Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", ".."));

    private static string Q(string s) => s.Contains(' ') ? $"\"{s}\"" : s;

    public async Task<string> GetDumpJsonAsync(CancellationToken ct = default)
    {
        if (string.IsNullOrWhiteSpace(Classpath))
            throw new InvalidOperationException("Classpath is not set. Set JavaDumpClient.Classpath.");
        if (string.IsNullOrWhiteSpace(SeasonFile))
            throw new InvalidOperationException("SeasonFile is not set. Set JavaDumpClient.SeasonFile.");

        var psi = new ProcessStartInfo
        {
            FileName = JavaCmd,
            Arguments =
                $"-cp {Q(Classpath)} frc.robot.auto.ResolvedFixtureDump --file {Q(SeasonFile)}",
            WorkingDirectory = WorkingDirectory,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = true
        };

        using var p = Process.Start(psi) ?? throw new InvalidOperationException("Failed to start Java dump process.");
        var stdoutTask = p.StandardOutput.ReadToEndAsync();
        var stderrTask = p.StandardError.ReadToEndAsync();

        await p.WaitForExitAsync(ct);

        var stdout = await stdoutTask;
        var stderr = await stderrTask;

        if (p.ExitCode != 0)
            throw new InvalidOperationException($"Java dump failed (exit {p.ExitCode}).\n{stderr}");

        if (string.IsNullOrWhiteSpace(stdout))
            throw new InvalidOperationException($"Java dump returned empty output.\n{stderr}");

        return stdout;
    }
}
