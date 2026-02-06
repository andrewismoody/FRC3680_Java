using System;
using System.IO;
using System.Linq;
using AutoJsonBuilder.Helpers;

namespace AutoJsonBuilder.Tools
{
    // Simple CLI to run RandomAutoJsonGenerator from the command line.
    // Arguments:
    //   -s|--seed <int>       seed for RNG (default: current timestamp % 2^31)
    //   -o|--out <file>       output file path for single-file mode
    //   -d|--out-dir <dir>    output directory for multi-file mode (default: current directory)
    //   -n|--count <int>      number of files to generate (default: 1)
    //   -h|--help             show this message
    public static class RandomAutoJsonCli
    {
        public static int Main(string[] args)
        {
            try
            {
                if (args == null) args = Array.Empty<string>();
                if (args.Any(a => a == "-h" || a == "--help"))
                {
                    PrintHelp();
                    return 0;
                }

                int count = 1;
                long seed = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() & 0x7FFFFFFF;
                string? outFile = null;
                string outDir = Directory.GetCurrentDirectory();

                for (int i = 0; i < args.Length; i++)
                {
                    var a = args[i];
                    if (a == "-n" || a == "--count")
                    {
                        if (i + 1 < args.Length && int.TryParse(args[i + 1], out var v)) { count = Math.Max(1, v); i++; }
                    }
                    else if (a == "-s" || a == "--seed")
                    {
                        if (i + 1 < args.Length && long.TryParse(args[i + 1], out var v)) { seed = v; i++; }
                    }
                    else if (a == "-o" || a == "--out")
                    {
                        if (i + 1 < args.Length) { outFile = args[i + 1]; i++; }
                    }
                    else if (a == "-d" || a == "--out-dir")
                    {
                        if (i + 1 < args.Length) { outDir = args[i + 1]; i++; }
                    }
                }

                if (!string.IsNullOrWhiteSpace(outFile) && count > 1)
                {
                    Console.Error.WriteLine("When using -o/--out (single output file) do not specify count > 1. Use -d/--out-dir for multiple files.");
                    return 2;
                }

                if (!Directory.Exists(outDir))
                {
                    try { Directory.CreateDirectory(outDir); } catch (Exception ex) { Console.Error.WriteLine($"Failed to create out-dir '{outDir}': {ex.Message}"); return 3; }
                }

                if (!string.IsNullOrWhiteSpace(outFile))
                {
                    // single-file mode: generate once (use given seed)
                    var json = RandomAutoJsonGenerator.Generate((int)seed, writeToFile: true, outPath: outFile);
                    Console.WriteLine($"Wrote: {Path.GetFullPath(outFile)} (seed={seed})");
                    return 0;
                }

                // multi-file mode: generate `count` files into outDir using incremental seeds
                for (int i = 0; i < count; i++)
                {
                    var s = (int)((seed + i) & 0x7FFFFFFF);
                    var defaultName = $"random-auto-seed-{s}.json";
                    var path = Path.Combine(outDir, defaultName);
                    var json = RandomAutoJsonGenerator.Generate(s, writeToFile: true, outPath: path);
                    Console.WriteLine($"Wrote: {Path.GetFullPath(path)} (seed={s})");
                }

                return 0;
            }
            catch (Exception ex)
            {
                Console.Error.WriteLine($"Error: {ex.Message}");
                return 1;
            }
        }

        private static void PrintHelp()
        {
            Console.WriteLine("RandomAutoJsonCli - generate schema-valid random auto.json files");
            Console.WriteLine();
            Console.WriteLine("Usage:");
            Console.WriteLine("  RandomAutoJsonCli [-s seed] [-n count] [-d out-dir]");
            Console.WriteLine("  RandomAutoJsonCli -o out-file [-s seed]");
            Console.WriteLine();
            Console.WriteLine("Options:");
            Console.WriteLine("  -s, --seed <int>       RNG seed (default: current time)");
            Console.WriteLine("  -n, --count <int>      number of files to generate (default: 1)");
            Console.WriteLine("  -d, --out-dir <dir>    output directory for generated files (default: current directory)");
            Console.WriteLine("  -o, --out <file>       write single file to the specified path");
            Console.WriteLine("  -h, --help             show this help");
            Console.WriteLine();
            Console.WriteLine("Examples:");
            Console.WriteLine("  dotnet run --project tools/AutoJsonBuilder -- -n 5 -d ./out");
            Console.WriteLine("  dotnet run --project tools/AutoJsonBuilder -- -o ./random.json -s 42");
        }
    }
}
