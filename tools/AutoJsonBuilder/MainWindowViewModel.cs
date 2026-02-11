using System.Collections.ObjectModel;
using System.IO;
using System.Text.Json;
using System.Windows.Input;
using AutoJsonBuilder.Models;
using AutoJsonBuilder.Helpers; // <-- added
using System.Windows; // <-- added for Dispatcher
using System.Diagnostics; // <-- added for timing
using OxyPlot;
using System.ComponentModel; // <-- add near other usings
using System.Linq; // <-- replaced reflection/usings with Linq
using System.Reflection; // <-- added

namespace AutoJsonBuilder;

public sealed class MainWindowViewModel : NotifyBase
{
    // SINGLE FLOW: schema-aligned document only
    private AutoDefinitionModel _doc = new();

    // JSON options that produce schema-compliant property names (camelCase)
    private static readonly JsonSerializerOptions JsonOptionsCamel = new JsonSerializerOptions { PropertyNamingPolicy = JsonNamingPolicy.CamelCase };
    private static readonly JsonSerializerOptions JsonOptionsCamelIndented = new JsonSerializerOptions { PropertyNamingPolicy = JsonNamingPolicy.CamelCase, WriteIndented = true };

    public ObservableCollection<TreeItemVm> TreeRootItems { get; } = new();
    public ObservableCollection<string> ParamKeys { get; } = new();

    private string _selectionDetails = "";
    public string SelectionDetails { get => _selectionDetails; set => Set(ref _selectionDetails, value); }

    private string? _currentJsonPath;
    public string? CurrentJsonPath { get => _currentJsonPath; set => Set(ref _currentJsonPath, value); }

    public ICommand NewCommand { get; }
    public ICommand OpenCommand { get; }
    public ICommand SaveCommand { get; }    // <-- new
    public ICommand SaveAsCommand { get; }

    private string _debugLog = "";
    public string DebugLog { get => _debugLog; set => Set(ref _debugLog, value); }

    // params map used to resolve expressions for calculated display values
    private IDictionary<string, object> _paramsMap = new Dictionary<string, object>(StringComparer.OrdinalIgnoreCase);

    // NEW: loading indicator for long-running tree rebuilds (bind this to a spinner/overlay in the view)
    private bool _isLoading;
    public bool IsLoading { get => _isLoading; set => Set(ref _isLoading, value); }

    // Guard to prevent concurrent SaveAs prompts / double prompting.
    private int _isSavePromptActive = 0;

    // Suppress autosave/prompt while performing load/rehydrate work (increment for nesting).
    // When > 0, SavePromptOrAutoSaveAsync will be a no-op.
    private int _suppressAutoSaveDuringLoad = 0;

    // Helper used by save/prompt flow to check whether autosave is suppressed (e.g. during OpenJson).
    private bool IsAutosaveSuppressed => Thread.VolatileRead(ref _suppressAutoSaveDuringLoad) > 0;

    // In-memory buffer for log lines while disk writes are suppressed during load.
    private readonly List<string> _deferredLogLines = new();
    private readonly object _deferredLogLock = new();

    // Plot model exposed to the view (bound to PlotView in XAML)
    private PlotModel? _fieldPlotModel;
    public PlotModel? FieldPlotModel { get => _fieldPlotModel; set => Set(ref _fieldPlotModel, value); }

    // Simple palette used to assign a unique color to each fixture type (wraps if more types).
    private static readonly OxyColor[] _palette = new[]
    {
        OxyColors.SteelBlue, OxyColors.IndianRed, OxyColors.DarkGreen, OxyColors.Goldenrod,
        OxyColors.MediumPurple, OxyColors.CadetBlue, OxyColors.Chocolate, OxyColors.DarkMagenta
    };

    // evaluated lookup (param name -> evaluated value). Populated for each Rebuild.
    private readonly Dictionary<string, object?> _evaluatedLookup = new(StringComparer.OrdinalIgnoreCase);

    // resolved fixture positions keyed by "type:index" -> (x,y,z,rotation?)
    private readonly Dictionary<string, (double x, double y, double z, double? rot)> _resolvedFixturePositions
        = new(StringComparer.OrdinalIgnoreCase);

    // NEW: resolved target positions keyed by target array index -> (x,y,z,rotation?)
    private readonly Dictionary<int, (double x, double y, double z, double? rot)> _resolvedTargetPositions
        = new();

    // Series filter collection bound to the UI dropdown
    public ObservableCollection<SeriesFilterItem> SeriesFilters { get; } = new();

    // Small helper VM for each series checkbox
    public sealed class SeriesFilterItem : INotifyPropertyChanged
    {
        public string Title { get; }
        public OxyPlot.Series.Series SeriesRef { get; }
        private bool _isChecked;
        public bool IsChecked
        {
            get => _isChecked;
            set
            {
                if (_isChecked == value) return;
                _isChecked = value;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsChecked)));
            }
        }
        public SeriesFilterItem(string title, OxyPlot.Series.Series series, bool isChecked = true)
        {
            Title = title;
            SeriesRef = series;
            _isChecked = isChecked;
        }
        public event PropertyChangedEventHandler? PropertyChanged;
    }

    // canonical list reused by ParamsInitializer (keep in sync)
    private static readonly HashSet<string> _providedParamNames = new(StringComparer.OrdinalIgnoreCase)
    {
        "fieldSize","fieldCenter","frameSize","frameCenter","bumperWidth",
        "robotSize","robotCenter","startArea","redStartTranslation","redStartRotation",
        "startPadding","robotOffset","blueStartY1","blueStartY2","blueStartY3","blueStartPosition"
    };

    // Try to resolve an arbitrary "raw" value into a double.
    // Accepts numbers, boxed numeric types, numeric strings, or parameter name strings that exist in _evaluatedLookup.
    // Returns true and sets out value on success.
    private bool TryResolveDoubleFromObject(object? raw, out double value)
    {
        value = 0.0;
        try
        {
            if (raw is null) return false;

            // Handle JsonElement values left by System.Text.Json deserialization
            if (raw is JsonElement je)
            {
                switch (je.ValueKind)
                {
                    case JsonValueKind.Number:
                        if (je.TryGetDouble(out var d))
                        {
                            value = d;
                            return true;
                        }
                        break;
                    case JsonValueKind.String:
                        var sje = je.GetString();
                        if (!string.IsNullOrEmpty(sje) && double.TryParse(sje, System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var parsedJe))
                        {
                            value = parsedJe;
                            return true;
                        }
                        // fall through to string-handling below
                        raw = sje;
                        break;
                    case JsonValueKind.Null:
                        return false;
                    // arrays/objects fall through to best-effort handling below (likely unresolved)
                }
            }

            // unwrap nullable numeric (boxed)
            switch (raw)
            {
                case double d:
                    value = d; return true;
                case float f:
                    value = Convert.ToDouble(f); return true;
                case int i:
                    value = Convert.ToDouble(i); return true;
                case long l:
                    value = Convert.ToDouble(l); return true;
                case decimal m:
                    value = Convert.ToDouble(m); return true;
                case bool _:
                    return false; // don't coerce booleans
            }

            // string: numeric literal or param key or expression -> delegate to Java interop when non-numeric
            if (raw is string str)
            {
                // numeric literal?
                if (double.TryParse(str, System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var parsed))
                {
                    value = parsed; return true;
                }

                // If a param key exists and is already numeric, return it directly.
                if (_evaluatedLookup.TryGetValue(str, out var ev))
                {
                    if (ev is double dd) { value = dd; return true; }
                    if (ev is float ff) { value = Convert.ToDouble(ff); return true; }
                    if (ev is int ii) { value = Convert.ToDouble(ii); return true; }
                    if (ev is long ll) { value = Convert.ToDouble(ll); return true; }
                    // If the param value is a string (expression) prefer evaluating that via Java interop
                    if (ev is string eset)
                    {
                        str = eset;
                    }
                    // otherwise fall through and treat original 'str' as expression to send to Java
                }

                // Build JSON payload for AutoExpr: { "expr": "<expr>", "params": { ... } }
                try
                {
                    var payload = new Dictionary<string, object?>(StringComparer.OrdinalIgnoreCase)
                    {
                        ["expr"] = str,
                        ["params"] = _evaluatedLookup ?? new Dictionary<string, object?>()
                    };
                    var options = new JsonSerializerOptions { PropertyNamingPolicy = JsonNamingPolicy.CamelCase, DefaultIgnoreCondition = System.Text.Json.Serialization.JsonIgnoreCondition.WhenWritingNull };
                    var inputJson = JsonSerializer.Serialize(payload, options);

                    // Use the interop (classpath JVM). Allow a bit more time for JVM startup.
                    var resp = FixtureResolverInterop.EvaluateExpression(inputJson, timeoutMs: 2000);
                    if (!string.IsNullOrWhiteSpace(resp))
                    {
                        using var jd = JsonDocument.Parse(resp);
                        if (jd.RootElement.TryGetProperty("value", out var vEl) && vEl.ValueKind == JsonValueKind.Number && vEl.TryGetDouble(out var dv))
                        {
                            value = dv;
                            return true;
                        }
                    }
                }
                catch (Exception ex)
                {
                    Log($"TryResolveDoubleFromObject: EvaluateExpression failed: {ex.Message}");
                }

                return false;
            }

            // If it's a wrapper with a 'Raw' property (ParamValue), attempt to get it
            var t = raw.GetType();
            var rawProp = t.GetProperty("Raw");
            if (rawProp != null)
            {
                var inner = rawProp.GetValue(raw);
                return TryResolveDoubleFromObject(inner, out value);
            }

            // Best-effort: if it's an IEnumerable<object> prefer first numeric element
            if (raw is System.Collections.IEnumerable ie)
            {
                foreach (var el in ie)
                {
                    if (TryResolveDoubleFromObject(el, out var v))
                    {
                        value = v;
                        return true;
                    }
                    // only try the first few items minimally
                    break;
                }
            }
        }
        catch { /* best-effort */ }

        return false;
    }

    // Try to resolve a fixture's numeric position + rotation.
    // Returns true if X/Y were resolved (Z/rotation optional).
    private bool TryGetFixturePosition(FixtureSchemaModel f, out double x, out double y, out double z, out double? rot)
    {
        x = 0; y = 0; z = 0; rot = null;
        if (f == null) return false;

        // Only use the resolved positions provided by the external Java resolver.
        // Do NOT attempt to evaluate expressions or derivedFrom locally for plotting.
        try
        {
            var key = $"{f.Type}:{f.Index}";
            if (_resolvedFixturePositions.TryGetValue(key, out var pos))
            {
                x = pos.x; y = pos.y; z = pos.z; rot = pos.rot;
                return true;
            }
        }
        catch { /* best-effort */ }

        return false;
    }

    // Re-add EvaluateParamsOnly (was accidentally removed) -----------------------------------------
    // Populate _evaluatedLookup from _doc.Params (best-effort, local conversion).
    // Used by tree building / UI editor options. This intentionally does NOT run external Java.
    private void EvaluateParamsOnly()
    {
        _evaluatedLookup.Clear();

        try
        {
            // Best-effort local conversion of raw param values into evaluated lookup used by UI previews.
            // This intentionally does not run external Java expression evaluation; it preserves numbers, strings, arrays.
            foreach (var kv in _doc.Params)
            {
                var name = kv.Key;
                var wrapper = kv.Value;
                object? raw = wrapper?.Raw;

                if (raw == null)
                {
                    _evaluatedLookup[name] = null;
                    continue;
                }

                // numbers -> double
                if (raw is double d) { _evaluatedLookup[name] = d; continue; }
                if (raw is float f) { _evaluatedLookup[name] = Convert.ToDouble(f); continue; }
                if (raw is int i) { _evaluatedLookup[name] = Convert.ToDouble(i); continue; }
                if (raw is long l) { _evaluatedLookup[name] = Convert.ToDouble(l); continue; }
                if (raw is decimal dec) { _evaluatedLookup[name] = Convert.ToDouble(dec); continue; }

                // string -> keep as string
                if (raw is string s) { _evaluatedLookup[name] = s; continue; }

                // array -> preserve as List<object?> with numeric conversions where obvious
                if (raw is IList<object> list)
                {
                    var outList = new List<object?>();
                    foreach (var el in list)
                    {
                        if (el == null) outList.Add(null);
                        else if (el is double dd) outList.Add(dd);
                        else if (el is float ff) outList.Add(Convert.ToDouble(ff));
                        else if (el is int ii) outList.Add(Convert.ToDouble(ii));
                        else if (el is long ll) outList.Add(Convert.ToDouble(ll));
                        else if (el is string ss) outList.Add(ss);
                        else outList.Add(el);
                    }
                    _evaluatedLookup[name] = outList;
                    continue;
                }

                // fallback: keep raw as-is
                _evaluatedLookup[name] = raw;
            }
        }
        catch (Exception ex)
        {
            Log($"EvaluateParamsOnly failed: {ex.Message}");
        }
    }
    // -----------------------------------------------------------------------------------------------

    // New helpers: unit conversion and rotation normalization used for plotting translations.
    private static double ConvertLengthToMeters(double value, string? units)
    {
        if (string.IsNullOrWhiteSpace(units)) return value * 0.0254; // default inches -> meters
        var u = units.Trim().ToLowerInvariant();
        if (u == "m" || u == "meter" || u == "meters" || u == "metre" || u == "metres") return value;
        if (u == "in" || u == "inch" || u == "inches") return value * 0.0254;
        if (u == "ft" || u == "foot" || u == "feet") return value * 0.3048;
        // unknown => treat as inches for backwards compatibility
        return value * 0.0254;
    }

    private static double? NormalizeRotationToDegrees(double? value, string? units)
    {
        if (value == null) return null;
        if (string.IsNullOrWhiteSpace(units)) return value * 180.0 / Math.PI; // assume radians if not specified
        var u = units.Trim().ToLowerInvariant();
        if (u.Contains("deg")) return value;
        // otherwise assume radians
        return value * 180.0 / Math.PI;
    }

    // Rebuild the PlotModel from current _doc fixtures and targets. Groups fixtures by Type and plots XY points.
    private void UpdatePlotModel()
    {
        try
        {
            // Ensure evaluated params/local lookup is fresh for any NumberOrParam translation values.
            try { EvaluateParamsOnly(); } catch { /* best-effort */ }

            // ensure fixture resolution is up-to-date before plotting.
            try { ResolveFixturesOnly(); } catch { /* best-effort */ }

            // NEW: log per-target eligibility (why some targets may not be plotted)
            try { LogTargetPlotEligibility(); } catch { /* best-effort */ }

            // Delegate to helper that constructs the full PlotModel. Helper returns a PlotModel and accepts a log callback.
            var pm = PlotHelper.BuildFieldPlot(_doc, _evaluatedLookup, _resolvedFixturePositions, _resolvedTargetPositions, _palette, Log);
            FieldPlotModel = pm;

            // Refresh the SeriesFilters UI so user can show/hide series
            RefreshSeriesFilters(pm);
        }
        catch (Exception ex)
        {
            Log($"UpdatePlotModel failed: {ex.Message}");
        }
    }

    // Resolve fixtures using the Java ResolvedFixtureDump (or other configured interop).
    // Populates _resolvedFixturePositions used exclusively by plotting code.
    private void ResolveFixturesOnly()
    {
        _resolvedFixturePositions.Clear();
        _resolvedTargetPositions.Clear(); // NEW: clear previous resolved targets
        try
        {
            string? outputJson = null;
            // If we have a saved file path, prefer calling the Java tool with that path (use the actual file).
            // Call ResolveFromFile unconditionally when CurrentJsonPath != null so the Java loader sees the exact file.
            if (!string.IsNullOrWhiteSpace(CurrentJsonPath))
            {
                Log($"ResolveFixturesOnly: attempting ResolveFromFile('{CurrentJsonPath}')");
                var fileOutput = FixtureResolverInterop.ResolveFromFile(CurrentJsonPath);
                // Diagnostic: log whether ResolveFromFile produced any output and its size
                if (string.IsNullOrWhiteSpace(fileOutput))
                {
                    Log("ResolveFixturesOnly: ResolveFromFile returned empty/null output.");
                }
                else
                {
                    Log($"ResolveFixturesOnly: ResolveFromFile returned {fileOutput.Length} bytes.");
                }
                if (!string.IsNullOrWhiteSpace(fileOutput))
                {
                    // If ResolveFromFile produced output, examine whether it contains resolved 'targets'.
                    // If it does not, prefer asking the Java evaluator to process the full in-memory document
                    // so that targets are evaluated the same way fixtures are.
                    try
                    {
                        using var pf = JsonDocument.Parse(fileOutput);
                        var myroot = pf.RootElement;
                        var hasTargets = myroot.TryGetProperty("targets", out var tEl) && tEl.ValueKind == JsonValueKind.Array && tEl.GetArrayLength() > 0;
                        Log($"ResolveFixturesOnly: ResolveFromFile contains 'targets' = {hasTargets}.");
                        if (hasTargets)
                        {
                            outputJson = fileOutput;
                        }
                        else
                        {
                            // Fall back to in-memory Resolve which sends the full document (fixtures + targets) to the evaluator.
                            var docJson = JsonHelper.SerializeCamel(_doc);
                            Log("ResolveFixturesOnly: resolver file output missing 'targets' — calling Resolve(in-memory) to evaluate targets.");
                            var memOutput = FixtureResolverInterop.Resolve(docJson);
                            if (string.IsNullOrWhiteSpace(memOutput))
                                Log("ResolveFixturesOnly: Resolve(in-memory) returned empty/null output.");
                            else
                                Log($"ResolveFixturesOnly: Resolve(in-memory) returned {memOutput.Length} bytes.");
                            if (!string.IsNullOrWhiteSpace(memOutput))
                            {
                                // prefer memOutput if it contains targets, else keep fileOutput
                                try
                                {
                                    using var pm = JsonDocument.Parse(memOutput);
                                    var mroot = pm.RootElement;
                                    var memHasTargets = mroot.TryGetProperty("targets", out var mtEl) && mtEl.ValueKind == JsonValueKind.Array && mtEl.GetArrayLength() > 0;
                                    Log($"ResolveFixturesOnly: Resolve(in-memory) contains 'targets' = {memHasTargets}.");
                                    if (memHasTargets)
                                    {
                                        outputJson = memOutput;
                                    }
                                    else
                                    {
                                        // neither contains targets: fall back to fileOutput (best-effort)
                                        outputJson = fileOutput;
                                    }
                                }
                                catch
                                {
                                    // parse failed: keep fileOutput
                                    outputJson = fileOutput;
                                }
                            }
                            else
                            {
                                // memOutput empty: fall back to fileOutput
                                outputJson = fileOutput;
                            }
                        }
                    }
                    catch
                    {
                        // If parsing fileOutput failed for any reason, fall back to trying in-memory resolve
                        var docJson = JsonHelper.SerializeCamel(_doc);
                        outputJson = FixtureResolverInterop.Resolve(docJson);
                        Log($"ResolveFixturesOnly: fallback Resolve(in-memory) returned {(outputJson == null ? "null/empty" : outputJson.Length.ToString() + " bytes")}.");
                        if (string.IsNullOrWhiteSpace(outputJson))
                        {
                            // final fallback to raw file output even if unparsed
                            outputJson = fileOutput;
                        }
                    }
                }
                else
                {
                    Log($"ResolveFixturesOnly: ResolveFromFile failed for '{CurrentJsonPath}', falling back to in-memory JSON");
                }
            }

            // Fallback: send in-memory JSON (used for unsaved docs or if ResolveFromFile failed)
            if (string.IsNullOrWhiteSpace(outputJson))
            {
                // use helper to produce camel-case JSON
                var docJson = JsonHelper.SerializeCamel(_doc);
                outputJson = FixtureResolverInterop.Resolve(docJson);
                Log($"ResolveFixturesOnly: final Resolve(in-memory) returned {(outputJson == null ? "null/empty" : outputJson.Length.ToString() + " bytes")}.");
            }
            if (string.IsNullOrWhiteSpace(outputJson)) return;

            // Diagnostic: record whether the final output contains targets before parsing
            try
            {
                using var quickCheck = JsonDocument.Parse(outputJson);
                var hasTargetsFinal = quickCheck.RootElement.TryGetProperty("targets", out var tt) && tt.ValueKind == JsonValueKind.Array && tt.GetArrayLength() > 0;
                Log($"ResolveFixturesOnly: final resolver output contains 'targets' = {hasTargetsFinal}.");
                if (!hasTargetsFinal)
                {
                    // persist the output for offline inspection (best-effort)
                    try
                    {
                        var dbgPath = Path.Combine(AppContext.BaseDirectory, "resolver-last-output.json");
                        File.WriteAllText(dbgPath, outputJson);
                        Log($"ResolveFixturesOnly: dumped resolver output to '{dbgPath}' for inspection.");
                    }
                    catch { /* ignore disk failures */ }
                }
            }
            catch (Exception ex)
            {
                Log($"ResolveFixturesOnly: failed quick-check parse of resolver output: {ex.Message}");
            }

            using var doc = JsonDocument.Parse(outputJson);
            var root = doc.RootElement;

            // helper: parse numeric fields tolerant of strings
            static bool TryGetDouble(JsonElement el, string name, out double value)
            {
                value = 0;
                if (el.ValueKind != JsonValueKind.Object) return false;
                if (!el.TryGetProperty(name, out var prop)) return false;
                if (prop.ValueKind == JsonValueKind.Number && prop.TryGetDouble(out var d)) { value = d; return true; }
                if (prop.ValueKind == JsonValueKind.String && double.TryParse(prop.GetString(), System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var sd)) { value = sd; return true; }
                return false;
            }

            // helper: read rotation and normalize to degrees (assume radians if units omitted)
            static double? ReadRotationNormalizedToDegrees(JsonElement el)
            {
                string[] rotNames = new[] { "rotation", "rot" };
                string[] unitNames = new[] { "rotationUnits", "rotUnits", "units" };
                foreach (var rn in rotNames)
                {
                    if (el.TryGetProperty(rn, out var rEl))
                    {
                        if (rEl.ValueKind == JsonValueKind.Number && rEl.TryGetDouble(out var rVal))
                        {
                            string? units = null;
                            foreach (var un in unitNames)
                                if (el.TryGetProperty(un, out var uEl) && uEl.ValueKind == JsonValueKind.String) { units = uEl.GetString(); break; }
                            if (!string.IsNullOrWhiteSpace(units) && units.Trim().Equals("degrees", StringComparison.OrdinalIgnoreCase)) return rVal;
                            // otherwise assume radians -> convert to degrees
                            return rVal * 180.0 / Math.PI;
                        }
                        if (rEl.ValueKind == JsonValueKind.Null) return null;
                    }
                }
                return null;
            }

            if (root.TryGetProperty("fixtures", out var fixturesEl))
            {
                if (fixturesEl.ValueKind == JsonValueKind.Object)
                {
                    foreach (var prop in fixturesEl.EnumerateObject())
                    {
                        try
                        {
                            var key = prop.Name;
                            var f = prop.Value;
                            if (!TryGetDouble(f, "x", out var x)) continue;
                            if (!TryGetDouble(f, "y", out var y)) continue;
                            var z = 0.0;
                            if (!TryGetDouble(f, "z", out var tmpZ)) tmpZ = 0.0; else z = tmpZ;
                            double? rotDeg = ReadRotationNormalizedToDegrees(f);
                            _resolvedFixturePositions[key] = (x, y, z, rotDeg);
                        }
                        catch { /* skip malformed entry */ }
                    }
                }
                else if (fixturesEl.ValueKind == JsonValueKind.Array)
                {
                    foreach (var f in fixturesEl.EnumerateArray())
                    {
                        try
                        {
                            var key = f.GetProperty("key").GetString() ?? "";
                            var x = f.GetProperty("x").GetDouble();
                            var y = f.GetProperty("y").GetDouble();
                            var z = 0.0;
                            if (f.TryGetProperty("z", out var zEl) && zEl.ValueKind == JsonValueKind.Number) z = zEl.GetDouble();
                            double? rotDeg = ReadRotationNormalizedToDegrees(f);
                            _resolvedFixturePositions[key] = (x, y, z, rotDeg);
                        }
                        catch { /* skip malformed entry */ }
                    }
                }
            }

            // --- NEW: parse targets array from resolver output (best-effort) ---
            if (root.TryGetProperty("targets", out var targetsEl) && targetsEl.ValueKind == JsonValueKind.Array)
            {
                int idx = 0;
                foreach (var tEl in targetsEl.EnumerateArray())
                {
                    try
                    {
                        double x = double.NaN, y = double.NaN, z = 0.0;
                        double? rotDeg = null;
                        bool found = false;

                        // direct x/y at root of target element
                        if (tEl.ValueKind == JsonValueKind.Object)
                        {
                            if (TryGetDouble(tEl, "x", out var tx) && TryGetDouble(tEl, "y", out var ty))
                            {
                                x = tx; y = ty; found = true;
                                if (TryGetDouble(tEl, "z", out var tz)) z = tz;
                                rotDeg = ReadRotationNormalizedToDegrees(tEl);
                            }
                            else if (tEl.TryGetProperty("translation", out var trEl) && trEl.ValueKind == JsonValueKind.Object)
                            {
                                // translation.position array [x,y,z] or fields x/y
                                if (trEl.TryGetProperty("position", out var posEl) && posEl.ValueKind == JsonValueKind.Array && posEl.GetArrayLength() >= 2)
                                {
                                    // parse position[0], position[1], optional position[2]
                                    bool px = false, py = false;
                                    try
                                    {
                                        var p0 = posEl[0];
                                        if (p0.ValueKind == JsonValueKind.Number && p0.TryGetDouble(out var dv0)) { x = dv0; px = true; }
                                        else if (p0.ValueKind == JsonValueKind.String && double.TryParse(p0.GetString(), System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var sv0)) { x = sv0; px = true; }

                                        var p1 = posEl[1];
                                        if (p1.ValueKind == JsonValueKind.Number && p1.TryGetDouble(out var dv1)) { y = dv1; py = true; }
                                        else if (p1.ValueKind == JsonValueKind.String && double.TryParse(p1.GetString(), System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var sv1)) { y = sv1; py = true; }

                                        if (posEl.GetArrayLength() >= 3)
                                        {
                                            var p2 = posEl[2];
                                            if (p2.ValueKind == JsonValueKind.Number && p2.TryGetDouble(out var dv2)) z = dv2;
                                            else if (p2.ValueKind == JsonValueKind.String && double.TryParse(p2.GetString(), System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var sv2)) z = sv2;
                                        }

                                        if (px && py) found = true;
                                    }
                                    catch { /* ignore per-entry parse errors */ }
                                }
                                // fallback to x/y fields inside translation object
                                if (!found && TryGetDouble(trEl, "x", out var tx2) && TryGetDouble(trEl, "y", out var ty2))
                                {
                                    x = tx2; y = ty2; found = true;
                                    if (TryGetDouble(trEl, "z", out var tz2)) z = tz2;
                                }
                                rotDeg = ReadRotationNormalizedToDegrees(trEl);
                            }
                        }

                        if (found)
                        {
                            _resolvedTargetPositions[idx] = (x, y, z, rotDeg);

                            // inject numeric translation into in-memory _doc.Targets so PlotHelper sees numeric values.
                            // Best-effort: only mutate when index maps to an existing target object.
                            if (idx >= 0 && idx < _doc.Targets.Count)
                            {
                                try
                                {
                                    var existingTarget = _doc.Targets[idx];
                                    existingTarget.Translation = new TranslationModel
                                    {
                                        Position = new System.Collections.Generic.List<object> { x, y, z },
                                        PositionUnits = "m",
                                        Rotation = rotDeg,
                                        RotationUnits = rotDeg.HasValue ? "degrees" : null
                                    };
                                }
                                catch { /* best-effort only */ }
                            }
                        }
                    }
                    catch { /* skip malformed target entry */ }
                    idx++;
                }
            }
        }
        catch (Exception ex)
        {
            Log($"ResolveFixturesOnly failed: {ex.Message}");
        }
    }

    public void Log(string message)
    {
        var line = $"{DateTime.Now:HH:mm:ss.fff} {message}\n";

        // If a load is in progress, buffer all log lines (do NOT update DebugLog/UI to avoid many small UI updates).
        if (Thread.VolatileRead(ref _suppressAutoSaveDuringLoad) > 0)
        {
            lock (_deferredLogLock) { _deferredLogLines.Add(line); }
            return;
        }

        // Normal case: update UI log (single update) and attempt disk append.
        DebugLog += line;
        try
        {
            var path = Path.Combine(AppContext.BaseDirectory, "autobuilder-debug.log");
            File.AppendAllText(path, line);
        }
        catch
        {
            // ignore file logging failures; UI log still should work
        }
    }

    // Flush buffered log lines to disk in a single append (async). Safe to call when suppression cleared.
    private async Task FlushBufferedLogsAsync()
    {
        string[] snapshot;
        lock (_deferredLogLock)
        {
            if (_deferredLogLines.Count == 0) return;
            snapshot = _deferredLogLines.ToArray();
            _deferredLogLines.Clear();
        }

        try
        {
            var path = Path.Combine(AppContext.BaseDirectory, "autobuilder-debug.log");
            await File.AppendAllTextAsync(path, string.Concat(snapshot)).ConfigureAwait(false);
        }
        catch
        {
            // ignore disk failures during flush; the UI still contains the log
        }
    }

    // Apply buffered log lines to the in-memory UI log in a single update
    private void ApplyBufferedLogsToUi()
    {
        string[] snapshot;
        lock (_deferredLogLock)
        {
            if (_deferredLogLines.Count == 0) return;
            snapshot = _deferredLogLines.ToArray();
            _deferredLogLines.Clear();
        }
        // Single string append to avoid many small UI updates
        var combined = string.Concat(snapshot);
        DebugLog += combined;
    }

    public MainWindowViewModel()
    {
        NewCommand = new RelayCommand(() => NewSeason());
        OpenCommand = new RelayCommand(async () => await OpenJsonAsync());
        SaveCommand = new RelayCommand(async () => await SaveJsonAsync()); // <-- new
        SaveAsCommand = new RelayCommand(async () => await SaveAsJsonAsync());

        Log("MainWindowViewModel ctor reached.");

        // Hook model-changed callback: debounced by TreeHelper or triggered immediately by UI actions.
        TreeHelper.OnModelChangedCallback = () =>
        {
            // fire-and-forget save/prompt
            _ = SavePromptOrAutoSaveAsync();
        };

        NewSeason();
    }

    public void SetSelectedTreeItem(TreeItemVm? item)
    {
        if (item == null)
        {
            SelectionDetails = "";
            return;
        }

        // Prefer a short description header for all selections
        var descHeader = GetNodeDescription(item);

        // Determine the JSON payload to show:
        // - Field nodes => show { "<fieldName>": <value> }
        // - Section nodes (no model, top-level title) => show the corresponding collection from _doc
        // - Object/model nodes => show the model itself (KeyValuePair entries, PoseModel, EventModel, etc.)
        object? payload = null;

        if (item.Kind == TreeItemKind.Field)
        {
            // show the single field as a small JSON object { "<title>": value }
            var fieldName = item.Title ?? "value";
            var dict = new Dictionary<string, object?> { [fieldName] = item.Value };
            payload = dict;
        }
        else if (item.Model != null)
        {
            payload = item.Model;
        }
        else
        {
            // Top-level section click — map title -> _doc collection
            var title = (item.Title ?? "").Trim().ToLowerInvariant();
            switch (title)
            {
                case "params": payload = _doc.Params; break;
                case "poses": payload = _doc.Poses; break;
                case "fixtures": payload = _doc.Fixtures; break;
                case "targets": payload = _doc.Targets; break;
                case "events": payload = _doc.Events; break;
                case "sequences": payload = _doc.Sequences; break;
                case "modules": payload = _doc.Modules; break;
                case "groups": payload = _doc.Groups; break;
                case "travelgroups": payload = _doc.TravelGroups; break;
                case "locations": payload = _doc.Locations; break;
                case "positions": payload = _doc.Positions; break;
                case "actions": payload = _doc.Actions; break;
                default: payload = null; break;
            }
        }

        if (payload == null)
        {
            // fallback to description or title
            SelectionDetails = string.IsNullOrWhiteSpace(descHeader) ? (item.Title ?? "") : descHeader;
            return;
        }

        var json = JsonSerializer.Serialize(payload, new JsonSerializerOptions { WriteIndented = true });
        SelectionDetails = string.IsNullOrWhiteSpace(descHeader) ? json : $"{descHeader}\n\n{json}";
    }

    // Return a short description for the selected node to explain what the node represents.
    // Covers common model types and top-level section titles. Keep descriptions concise for the UI.
    private string GetNodeDescription(TreeItemVm? item)
    {
        if (item == null) return "";
        // If the VM holds a model object, prefer type-based descriptions
        var model = item.Model;
        if (model is null)
        {
            // Top-level section titles
            switch ((item.Title ?? "").Trim().ToLowerInvariant())
            {
                case "params":
                    return "params — Named values (numbers, arrays, or parameter expressions) referenced by other fields and expressions.";
                case "poses":
                    return "poses — Definitions of named poses (group, location, index, position and action) used by events and targets.";
                case "fixtures":
                    return "fixtures — Field fixtures: physical points on the field, either explicit translations or derived from other fixtures.";
                case "targets":
                    return "targets — Targets to pursue during auto: can be a translation, a fixture reference, a measurement, or a module state.";
                case "events":
                    return "events — Timing or trigger events that cause sequences to execute targets.";
                case "sequences":
                    return "sequences — Ordered lists of event names composing higher-level actions.";
                case "modules":
                case "groups":
                case "locations":
                case "positions":
                case "actions":
                case "travelgroups":
                    return $"{item.Title} — A list of labels used elsewhere in the document (referenced by poses, targets, and UI dropdowns).";
                default:
                    return "";
            }
        }

        // Type-based descriptions (models come from AutoJsonBuilder.Models)
        if (model is AutoDefinitionModel)
            return "Document root — the top-level auto definition for a season (season, version, description, and arrays of poses/events/etc.).";
        if (model is PoseModel)
            return "Pose — describes a named pose: group, location, index, position and action. Poses are referenced by events and targets.";
        if (model is EventModel)
            return "Event — a trigger (await/time) that will execute a target; contains pose/time/trigger settings.";
        if (model is FixtureSchemaModel)
            return "Fixture — a field fixture definition; either has an explicit translation or is derived from other fixtures.";
        if (model is TargetSchemaModel)
            return "Target — defines what to do for a module (or fixture): translation, fixture ref, measurement, or module state.";
        if (model is SequenceModel)
            return "Sequence — ordered list of events executed as a grouped action.";
        if (model is TranslationModel)
            return "Translation — X/Y/Z coordinates and units describing a physical location on the field.";
        if (model is FixtureRefModel)
            return "Fixture reference — refers to an existing fixture by type and index.";
        if (model is DerivedFromModel)
            return "Derived translation — computed translation derived from one or more fixtures (e.g. bisector, parallel).";
        if (model is AutoJsonBuilder.Models.ParamValue)
            return "Parameter — a named parameter used for expression evaluation; may be a scalar, an array, or an expression string.";

        // KeyValuePair used for params list entries
        var kvType = model.GetType();
        if (kvType.IsGenericType && kvType.GetGenericTypeDefinition() == typeof(System.Collections.Generic.KeyValuePair<,>))
        {
            // likely a param kvp
            return "Parameter entry — a named parameter key and its value. Parameters are referenced from expressions elsewhere.";
        }

        // fallback: for field-level items, show a short generic hint
        return $"{item.Title} — field within the document; edit the value to change the underlying document content used at runtime.";
    }

    private void UpdateParamsMap()
    {
        _paramsMap.Clear();
        // copy raw param values preserved in the model (double, string, List<object>, ...)
        foreach (var kv in _doc.Params)
        {
            _paramsMap[kv.Key] = kv.Value?.Raw ?? 0d;
        }
    }

    private void NewSeason()
    {
        // Clear previous debug output when creating a new document, then log the new session.
        DebugLog = "";
        Log("NewSeason: creating new document (seed).");

        _doc = new AutoDefinitionModel
        {
            // schema requires minItems 1 for most arrays; seed with sensible placeholders
            Season = "2025",
            Version = "1",
            Description = "",
        };

        // ensure params defaults/expressions are present for a freshly-created document
        ParamsInitializer.EnsureParams(_doc);

        // moved to PoseHelper
        PoseHelper.EnsurePoseBackingLists(_doc);

        // Ensure modules list exists and seed a sensible default so targets can reference it.
        if (_doc.Modules.Count == 0) _doc.Modules.Add("module1");

        var group = _doc.Groups[0];
        var location = _doc.Locations[0];
        var position = _doc.Positions[0];
        var action = _doc.Actions[0];
        var module = _doc.Modules.FirstOrDefault() ?? "module1";

        _doc.Poses.Add(new PoseModel { Group = group, Location = location, Index = 0, Position = position, Action = action });
        _doc.Events.Add(new EventModel { Name = "event1", Type = "await", Parallel = false, Pose = "pose1" });
        _doc.Fixtures.Add(new FixtureSchemaModel { Type = "fixture", Index = 0, Translation = new TranslationModel { Position = new() { 0d, 0d, 0d }, PositionUnits = "inches", Rotation = 0d, RotationUnits = "degrees" } });
        _doc.Targets.Add(new TargetSchemaModel { Module = module, Translation = new TranslationModel { Position = new() { 0d, 0d, 0d }, PositionUnits = "inches" } });

        // Ensure sequences backing lists exist
        TreeHelper.EnsureSequenceBackingLists(_doc);
        _doc.Sequences.Add(new SequenceModel { Name = "seq1", Events = new() { "event1" } });

        // ensure names exist (moved)
        PoseHelper.EnsurePoseNames(_doc);

        // update params map for UI calculations
        UpdateParamsMap();

        CurrentJsonPath = null;
        _ = RebuildTreeAsync(); // fire-and-forget so ctor/command remains sync-friendly

        Log("NewSeason: document initialized (unsaved).");
    }

    public string Season
    {
        get => _doc.Season ?? "";
        set
        {
            if (_doc.Season == value) return;
            _doc.Season = value;
            RebuildTree(); // update tree to reflect new season value
        }
    }

    public async Task SaveToPathAsync(string path)
    {
        // delegate serialization + write to FileHelper
        await FileHelper.WriteAutoJsonAsync(path, _doc);
        CurrentJsonPath = path;

        // update root label etc.
        await RebuildTreeAsync();

        Log($"SaveToPath: wrote '{path}'.");
    }

    private async Task OpenJsonAsync()
    {
        // Use FileHelper to prompt and read
        var chosen = FileHelper.PromptOpenAutoJsonPath();
        if (chosen == null) return;

        // Clear previous debug output when opening a file so the log reflects the new load.
        DebugLog = "";
        // Suppress autosave while we perform the open sequence (rebuilds, helpers, etc).
        Interlocked.Increment(ref _suppressAutoSaveDuringLoad);
        try
        {
            var json = await FileHelper.ReadAllTextAsync(chosen);

            // Parse once to extract the heterogeneous params map (preserve arrays/strings/expressions)
            using (var doc = JsonDocument.Parse(json))
            {
                _paramsMap = ParamAwareConverters.BuildParamsMap(doc.RootElement);
            }

            // Deserialize the model using forgiving dictionary converter for numeric params where the schema expects numbers.
            var opts = new JsonSerializerOptions { PropertyNameCaseInsensitive = true };
            opts.Converters.Add(new FlexibleDoubleDictionaryConverter()); // keeps older numeric semantics for model values
            _doc = JsonSerializer.Deserialize<AutoDefinitionModel>(json, opts)
                   ?? throw new InvalidOperationException("Invalid JSON file.");

            // Ensure params defaults/expressions are present after load (fills missing keys without overwriting)
            ParamsInitializer.EnsureParams(_doc);

            // moved
            PoseHelper.EnsurePoseBackingLists(_doc);

            // Ensure module list exists and absorb any module names referenced by targets
            _doc.Modules ??= new System.Collections.Generic.List<string>();
            // Collect target module names in document order (preserve existing modules then append missing referenced ones)
            foreach (var tgt in _doc.Targets)
            {
                var m = tgt?.Module;
                if (string.IsNullOrWhiteSpace(m)) continue;
                if (!_doc.Modules.Contains(m))
                    _doc.Modules.Add(m);
            }

            // ensure pose names are consistent after load
            PoseHelper.EnsurePoseNames(_doc);

            // Ensure sequences have their backing lists (events/start1/2/3)
            TreeHelper.EnsureSequenceBackingLists(_doc);

            // If deserialization produced different params representation, ensure _paramsMap is at least populated from _doc
            if (_paramsMap.Count == 0) UpdateParamsMap();

            CurrentJsonPath = chosen;
            await RebuildTreeAsync();

            // note: do NOT perform autosave or disk writes while suppression is active.
            Log($"OpenJson: loaded (in-memory) '{chosen}'.");
        }
        finally
        {
            // always clear suppression so subsequent model-changed callbacks will perform autosave/prompt
            Interlocked.Decrement(ref _suppressAutoSaveDuringLoad);

            // apply buffered logs to the in-memory DebugLog in one UI update (cheap).
            try { ApplyBufferedLogsToUi(); } catch { /* best-effort only */ }
        }
    }

    private async Task SaveAsJsonAsync()
    {
        var suggested = Path.GetFileName(CurrentJsonPath) ?? "auto.json";
        var chosen = FileHelper.PromptSaveAutoJsonPath(suggested);
        if (chosen == null) return;

        await FileHelper.WriteAutoJsonAsync(chosen, _doc);
        CurrentJsonPath = chosen;

        await RebuildTreeAsync(); // update root label

        Log($"SaveAsJson: wrote '{chosen}'.");
    }

    // Save to current path if present, otherwise fall back to Save As.
    private async Task SaveJsonAsync()
    {
        try
        {
            if (string.IsNullOrWhiteSpace(CurrentJsonPath))
            {
                await SaveAsJsonAsync();
                return;
            }

            // write current document to current path
            await SaveToPathAsync(CurrentJsonPath);
            Log($"Save: saved to '{CurrentJsonPath}'.");
        }
        catch (Exception ex)
        {
            Log($"Save failed: {ex.Message}");
        }
    }

    // ----- Tree helpers -----

    // NEW: delegate to TreeBuildHelper to construct the tree root (keeps VM small)
    private TreeItemVm BuildTreeRoot()
    {
        // ensure evaluated lookup is fresh for this document
        try { EvaluateParamsOnly(); } catch { /* best-effort */ }

        // Delegate actual construction to helper and supply delegates for VM-specific actions
        return TreeBuildHelper.BuildTreeRoot(
            _doc,
            ParamKeys,
            _paramsMap,
            // rebuildAsync
            async () => { await RebuildTreeAsync().ConfigureAwait(false); },
            // triggerModelChangedImmediate
            () => TreeHelper.TriggerModelChangedImmediate(),
            // focusNewUnder: (section, child) => TreeHelper.FocusNewUnder(TreeRootItems, section, child)
            (section, child) => TreeHelper.FocusNewUnder(TreeRootItems, section, child),
            // rename delegates
            async (oldKey, newKey) => await RenameParamAsync(oldKey, newKey).ConfigureAwait(false),
            async (oldName, newName) => await RenamePoseAsync(oldName, newName).ConfigureAwait(false),
            async (oldName, newName) => await RenameEventAsync(oldName, newName).ConfigureAwait(false),
            async (oldName, newName) => await RenameSequenceAsync(oldName, newName).ConfigureAwait(false),
            // commit pose caption
            async (node) => await CommitPoseCaptionAsync(node).ConfigureAwait(false)
        );
    }

    // RebuildTree now simply constructs and installs the tree synchronously (kept for compatibility).
    private void RebuildTree()
    {
        var sw = Stopwatch.StartNew();
        var prev = TreeHelper.GetViewState(TreeRootItems);

        var root = BuildTreeRoot();
        // ensure provided/default params appear non-editable/non-deletable in the tree
        ApplyProvidedParamFlags(root);

        sw.Stop();
        Log($"RebuildTree: BuildTreeRoot took {sw.ElapsedMilliseconds}ms");

        // install on UI-bound collection (must be done on UI thread)
        var swUi = Stopwatch.StartNew();
        TreeRootItems.Clear();
        TreeRootItems.Add(root);
        OnPropertyChanged(nameof(TreeRootItems));

        var swRefresh = Stopwatch.StartNew();
        TreeHelper.RefreshAllOptions(TreeRootItems);
        swRefresh.Stop();

        // restore previous expansion/selection where possible
        var sel = TreeHelper.ApplyViewState(TreeRootItems, prev);
        if (sel != null) SetSelectedTreeItem(sel);

        var swLog = Stopwatch.StartNew();
        LoggingHelper.LogAllNodeBindings(TreeRootItems, Log);
        swLog.Stop();
        // update plot to reflect newly built tree/document
        UpdatePlotModel();
        swUi.Stop();

        Log($"RebuildTree: UI apply {swUi.ElapsedMilliseconds}ms (Refresh {swRefresh.ElapsedMilliseconds}ms, LogAll {swLog.ElapsedMilliseconds}ms)");
    }

    // NEW: async wrapper so UI can show a loading indicator while tree is rebuilt.
    public async Task RebuildTreeAsync()
    {
        if (IsLoading) return;

        IsLoading = true;
        try
        {
            // yield to UI so bound spinner/overlay can render before heavy work begins
            await Task.Yield();

            // capture UI state before heavy background work
            var prevState = TreeHelper.GetViewState(TreeRootItems);

            // Build the tree on a background thread (the expensive part) and time it
            var swBuild = Stopwatch.StartNew();
            var root = await Task.Run(() => BuildTreeRoot());
            swBuild.Stop();

            // mark provided params as non-editable/non-deletable BEFORE installing the VM on the UI
            ApplyProvidedParamFlags(root);

            Log($"RebuildTreeAsync: BuildTreeRoot {swBuild.ElapsedMilliseconds}ms");

            // Marshal applying the constructed root to the UI thread and time UI-phase pieces
            long refreshMs = 0, logAllMs = 0, uiTotalMs = 0;
            if (Application.Current?.Dispatcher != null)
            {
                var swUi = Stopwatch.StartNew();
                Application.Current.Dispatcher.Invoke(() =>
                {
                    TreeRootItems.Clear();
                    TreeRootItems.Add(root);
                    OnPropertyChanged(nameof(TreeRootItems));

                    var swRefresh = Stopwatch.StartNew();
                    TreeHelper.RefreshAllOptions(TreeRootItems);
                    swRefresh.Stop();
                    refreshMs = swRefresh.ElapsedMilliseconds;

                    // restore previous expansion/selection where possible
                    var sel = TreeHelper.ApplyViewState(TreeRootItems, prevState);
                    if (sel != null) SetSelectedTreeItem(sel);

                    var swLog = Stopwatch.StartNew();
                    LoggingHelper.LogAllNodeBindings(TreeRootItems, Log);
                    swLog.Stop();
                    // update plot on UI thread so PlotModel binding sees the new model
                    UpdatePlotModel();
                    logAllMs = swLog.ElapsedMilliseconds;
                    swUi.Stop();
                    uiTotalMs = swUi.ElapsedMilliseconds;
                });
                Log($"RebuildTreeAsync: UI apply {uiTotalMs}ms (Refresh {refreshMs}ms, LogAll {logAllMs}ms)");
            }
            else
            {
                var swUi = Stopwatch.StartNew();
                TreeRootItems.Clear();
                TreeRootItems.Add(root);
                OnPropertyChanged(nameof(TreeRootItems));

                var swRefresh = Stopwatch.StartNew();
                TreeHelper.RefreshAllOptions(TreeRootItems);
                swRefresh.Stop();
                refreshMs = swRefresh.ElapsedMilliseconds;

                var sel = TreeHelper.ApplyViewState(TreeRootItems, prevState);
                if (sel != null) SetSelectedTreeItem(sel);

                var swLog = Stopwatch.StartNew();
                LoggingHelper.LogAllNodeBindings(TreeRootItems, Log);
                swLog.Stop();
                UpdatePlotModel();
                logAllMs = swLog.ElapsedMilliseconds;
                swUi.Stop();
                uiTotalMs = swUi.ElapsedMilliseconds;

                Log($"RebuildTreeAsync: UI apply {uiTotalMs}ms (Refresh {refreshMs}ms, LogAll {logAllMs}ms)");
            }
        }
        finally
        {
            IsLoading = false;
        }
    }

	// Public helper: commit pose caption edit (call this from the caption TextBox LostFocus)
	public async Task CommitPoseCaptionAsync(TreeItemVm? node)
	{
		if (node?.Model is not PoseModel pose) return;
		var newTitle = node.Title?.Trim() ?? "";
		var oldName = pose.Name ?? "";
		if (string.IsNullOrWhiteSpace(newTitle))
		{
			// revert to old name on invalid input
			node.Title = oldName;
			return;
		}
		if (newTitle == oldName) return;
		if (_doc.Poses.Any(x => string.Equals(x.Name, newTitle, StringComparison.OrdinalIgnoreCase)))
		{
			// collision: revert UI
			node.Title = oldName;
			return;
		}

		await RenamePoseAsync(oldName, newTitle);
		// ensure immediate save
		TreeHelper.TriggerModelChangedImmediate();
	}

    // Add this helper near other private helpers in the class
    private async Task RenameParamAsync(string oldKey, string newKey)
    {
        if (string.IsNullOrWhiteSpace(oldKey) || string.IsNullOrWhiteSpace(newKey) || oldKey == newKey) return;
        // validate
        if (!_doc.Params.ContainsKey(oldKey))
        {
            Log($"RenameParam: old key '{oldKey}' not found.");
            return;
        }
        if (_doc.Params.ContainsKey(newKey))
        {
            Log($"RenameParam: target key '{newKey}' already exists — abort.");
            return;
        }

        Log($"RenameParam: renaming '{oldKey}' -> '{newKey}'");
        var val = _doc.Params[oldKey];
        _doc.Params.Remove(oldKey);
        _doc.Params[newKey] = val!;

        // update lookup and UI lists
        UpdateParamsMap();
        ParamKeys.Clear();
        foreach (var k in _doc.Params.Keys.OrderBy(x => x)) ParamKeys.Add(k);

        // notify model changed so autosave triggers
        TreeHelper.TriggerModelChangedImmediate();

        // rebuild and focus the renamed node
        await RebuildTreeAsync();
        TreeHelper.FocusNewUnder(TreeRootItems, "params", newKey);
    }

    // Add this helper near other private helpers in the class
    private async Task RenamePoseAsync(string oldName, string newName)
    {
        if (string.IsNullOrWhiteSpace(oldName) || string.IsNullOrWhiteSpace(newName) || oldName == newName) return;
        var pose = _doc.Poses.FirstOrDefault(p => string.Equals(p.Name, oldName, StringComparison.OrdinalIgnoreCase));
        if (pose is null)
        {
            Log($"RenamePose: old pose '{oldName}' not found.");
            return;
        }
        if (_doc.Poses.Any(p => string.Equals(p.Name, newName, StringComparison.OrdinalIgnoreCase)))
        {
            Log($"RenamePose: target name '{newName}' already exists — abort.");
            return;
        }

        Log($"RenamePose: renaming '{oldName}' -> '{newName}'");
        // update the pose name
        pose.Name = newName;

        // update references (events reference pose names)
        foreach (var ev in _doc.Events)
        {
            if (string.Equals(ev.Pose, oldName, StringComparison.OrdinalIgnoreCase))
                ev.Pose = newName;
        }

        // ensure stable unique names if helper enforces any format
        PoseHelper.EnsurePoseNames(_doc);

        // trigger immediate save for the rename
        TreeHelper.TriggerModelChangedImmediate();

        // rebuild and focus renamed pose
        await RebuildTreeAsync();
        TreeHelper.FocusNewUnder(TreeRootItems, "poses", newName);
        Log($"RenamePose: completed '{oldName}' -> '{newName}'");
    }

    // Add this helper near other private helpers in the class
    private async Task RenameEventAsync(string oldName, string newName)
    {
        if (string.IsNullOrWhiteSpace(oldName) || string.IsNullOrWhiteSpace(newName) || oldName == newName) return;
        var ev = _doc.Events.FirstOrDefault(e => string.Equals(e.Name, oldName, StringComparison.OrdinalIgnoreCase));
        if (ev is null)
        {
            Log($"RenameEvent: old event '{oldName}' not found.");
            return;
        }
        if (_doc.Events.Any(e => string.Equals(e.Name, newName, StringComparison.OrdinalIgnoreCase)))
        {
            Log($"RenameEvent: target name '{newName}' already exists — abort.");
            return;
        }

        Log($"RenameEvent: renaming '{oldName}' -> '{newName}'");
        // update the event name
        ev.Name = newName;

        // Update any sequence references to the event name (events + start1/2/3)
        foreach (var seq in _doc.Sequences)
        {
            void ReplaceInList(System.Collections.Generic.List<string>? list)
            {
                if (list == null) return;
                for (int i = 0; i < list.Count; i++)
                {
                    if (string.Equals(list[i], oldName, StringComparison.OrdinalIgnoreCase))
                        list[i] = newName;
                }
            }

            ReplaceInList(seq.Events);
            ReplaceInList(seq.Start1);
            ReplaceInList(seq.Start2);
            ReplaceInList(seq.Start3);
        }

        // trigger immediate save for the rename
        TreeHelper.TriggerModelChangedImmediate();

        // rebuild and focus renamed event
        await RebuildTreeAsync();
        TreeHelper.FocusNewUnder(TreeRootItems, "events", newName);
        Log($"RenameEvent: completed '{oldName}' -> '{newName}'");
    }

    // Add this helper near other private helpers in the class
    private async Task RenameSequenceAsync(string oldName, string newName)
    {
        if (string.IsNullOrWhiteSpace(oldName) || string.IsNullOrWhiteSpace(newName) || oldName == newName) return;
        var seq = _doc.Sequences.FirstOrDefault(s => string.Equals(s.Name, oldName, StringComparison.OrdinalIgnoreCase));
        if (seq is null)
        {
            Log($"RenameSequence: old sequence '{oldName}' not found.");
            return;
        }
        if (_doc.Sequences.Any(s => string.Equals(s.Name, newName, StringComparison.OrdinalIgnoreCase)))
        {
            Log($"RenameSequence: target name '{newName}' already exists — abort.");
            return;
        }

        Log($"RenameSequence: renaming '{oldName}' -> '{newName}'");
        seq.Name = newName;

        // trigger immediate save for the rename
        TreeHelper.TriggerModelChangedImmediate();

        // rebuild and focus renamed sequence
        await RebuildTreeAsync();
        TreeHelper.FocusNewUnder(TreeRootItems, "sequences", newName);
        Log($"RenameSequence: completed '{oldName}' -> '{newName}'");
    }

    // Autosave helper: write current _doc to CurrentJsonPath if set (fire-and-forget from notifier).
    private async Task AutoSaveIfExistsAsync()
    {
        try
        {
            var path = CurrentJsonPath;
            if (string.IsNullOrWhiteSpace(path)) return;
            await FileHelper.WriteAutoJsonAsync(path, _doc);
            Log($"AutoSave: wrote '{path}'.");
        }
        catch (System.Exception ex)
        {
            Log($"AutoSave failed: {ex.Message}");
        }
    }

    // If there is no current path, prompt user with Save As; otherwise perform autosave.
    public async Task SavePromptOrAutoSaveAsync()
    {
        // If autosave is suppressed (e.g. during OpenJson flow) do nothing.
        if (IsAutosaveSuppressed)
        {
            // Log("Autosave suppressed during document load; skipping SavePromptOrAutoSaveAsync.");
            return;
        }

        // Prevent re-entrancy/double prompts.
        if (Interlocked.Exchange(ref _isSavePromptActive, 1) == 1)
        {
            // already handling a save/prompt
            return;
        }

        try
        {
            if (string.IsNullOrWhiteSpace(CurrentJsonPath))
            {
                // Ensure SaveAs dialog runs on UI thread
                if (Application.Current?.Dispatcher != null)
                {
                    await Application.Current.Dispatcher.InvokeAsync(async () =>
                    {
                        await SaveAsJsonAsync();
                    });
                }
                else
                {
                    await SaveAsJsonAsync();
                }
            }
            else
            {
                await AutoSaveIfExistsAsync();
            }
        }
        finally
        {
            Interlocked.Exchange(ref _isSavePromptActive, 0);
        }
    }

    // wrappers removed; VM now calls TreeHelper.Create* / TreeHelper.FocusNewUnder directly.

    private TreeItemVm BuildTranslationNode(TranslationModel tr)
    {
        // Ensure position array is present and exactly 3 elements (UI expects X/Y/Z children).
        tr.Position ??= new() { 0d, 0d, 0d };
        while (tr.Position.Count < 3) tr.Position.Add(0d);
        // Delegate to TreeHelper that builds a translation node with labeled position children and consistent behavior.
        return TreeHelper.CreateTranslationNode(tr, ParamKeys);
    }

    // Add these small helpers so the view can forward mouse clicks to the legend hit-test.
    // Call with mouse coordinates relative to the PlotView (e.GetPosition(plotView)).
    public bool HandlePlotMouseDown(System.Windows.Point plotViewPoint)
        => HandlePlotMouseDown(plotViewPoint.X, plotViewPoint.Y);

    public bool HandlePlotMouseDown(double plotViewX, double plotViewY)
    {
        // Legend interaction disabled: no-op and return false to indicate not handled.
        return false;
    }

	// Mouse move: show a small indicator where the mouse is (plotView-relative coords).
	// Returns true if indicator was shown (caller should call InvalidatePlot on the PlotView's Model if needed).
	public bool HandlePlotMouseMove(System.Windows.Point plotViewPoint, double radiusPx = 8)
		=> HandlePlotMouseMove(plotViewPoint.X, plotViewPoint.Y, radiusPx);

	public bool HandlePlotMouseMove(double plotViewX, double plotViewY, double radiusPx = 8)
	{
		if (FieldPlotModel == null) return false;
		try
		{
			 // Production: show a single cursor indicator using the chosen mapping
			PlotHelper.ShowCursorIndicator(FieldPlotModel, plotViewX, plotViewY, radiusPx);
			// caller (view) will InvalidatePlot; we return true to indicate we updated indicator.
			return true;
		}
		catch { return false; }
	}

	// Optional helper to clear indicator (e.g. on mouse leave)
	public void HandlePlotMouseLeave()
	{
		if (FieldPlotModel == null) return;
		PlotHelper.ClearCursorIndicator(FieldPlotModel);
		try { FieldPlotModel.InvalidatePlot(false); } catch { }
	}

    // Build or refresh the SeriesFilters list from the PlotModel
    private void RefreshSeriesFilters(PlotModel? pm)
    {
        SeriesFilters.Clear();
        if (pm == null) return;

        int idx = 0;
        foreach (var s in pm.Series)
        {
            var title = string.IsNullOrWhiteSpace(s.Title) ? $"series{idx}" : s.Title;

            // NOTE: do not exclude 'map' here — include it in the filters so users can toggle the background image.

            // prefer Series.IsVisible if available; assume visible by default otherwise
            bool isVisible = true;
            try
            {
                var prop = s.GetType().GetProperty("IsVisible");
                if (prop != null && prop.GetValue(s) is bool b) isVisible = b;
            }
            catch { }
            var item = new SeriesFilterItem(title, s, isVisible);
            // when IsChecked changes toggle the underlying series visibility and refresh plot
            item.PropertyChanged += (_, e) =>
            {
                if (e.PropertyName != nameof(SeriesFilterItem.IsChecked)) return;
                try
                {
                    // set Series.IsVisible if present (cross-version safe), else fallback to hiding via Opacity/MarkerFill alpha
                    var prop = s.GetType().GetProperty("IsVisible");
                    if (prop != null && prop.CanWrite)
                    {
                        prop.SetValue(s, item.IsChecked);
                    }
                    else
                    {
                        // fallback: attempt to dim/remove by setting MarkerFill alpha to 0 (best-effort)
                        if (s is OxyPlot.Series.ScatterSeries ss && ss.MarkerFill != null)
                        {
                            var c = ss.MarkerFill;
                            ss.MarkerFill = item.IsChecked ? OxyColor.FromArgb(255, c.R, c.G, c.B) : OxyColor.FromArgb(0, c.R, c.G, c.B);
                        }
                    }
                    // Also show/hide associated arrow annotations for this series (if any)
                    try { PlotHelper.SetSeriesAnnotationsVisibility(pm, s, item.IsChecked); } catch { }
                    // request redraw
                    try { pm.InvalidatePlot(false); } catch { }
                }
                catch { /* best-effort */ }
            };
            SeriesFilters.Add(item);
            idx++;
        }
    }

    // Walk the tree and set known VM properties on param entries that are provided by default.
    // This implementation now applies flags recursively to the param node and its child subtree
    // so array dimensions/elements cannot be added or removed for provided params.
    private void ApplyProvidedParamFlags(TreeItemVm? root)
    {
        if (root == null) return;

        // breadth-first search to find the "params" section node
        var q = new Queue<TreeItemVm>();
        q.Enqueue(root);
        TreeItemVm? paramsNode = null;
        while (q.Count > 0)
        {
            var node = q.Dequeue();
            if (string.Equals(node.Title, "params", StringComparison.OrdinalIgnoreCase))
            {
                paramsNode = node;
                break;
            }

            if (node.Children != null)
            {
                foreach (var ch in node.Children)
                    q.Enqueue(ch);
            }
        }

        if (paramsNode == null) return;
        if (paramsNode.Children == null) return;

        // recursive applier that disables add/remove/delete and nulls commands on node and descendants
        void ApplyFlagsToNode(TreeItemVm node)
        {
            try { node.IsCaptionEditable = false; } catch { }
            try { node.IsReadOnly = true; } catch { }

            // disable add/remove and delete affordances exposed by the VM
            try { node.IsDeletable = false; } catch { }
            try { node.HasRemove = false; } catch { }
            try { node.HasAdd = false; } catch { }

            // null common command properties if present on VM
            try { node.RemoveCommand = null; } catch { }
            try { node.AddCommand = null; } catch { }

            // reflection: null out other command-style properties templates may check for
            void NullifyCommandProp(object obj, string propName)
            {
                try
                {
                    var pi = obj.GetType().GetProperty(propName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.IgnoreCase);
                    if (pi != null && pi.CanWrite && typeof(ICommand).IsAssignableFrom(pi.PropertyType))
                        pi.SetValue(obj, null);
                }
                catch { }
            }

            var cmdNames = new[] { "DeleteCommand", "RemoveCommand", "DeleteCmd", "CmdDelete", "CmdRemove", "AddCommand", "AddCmd" };
            foreach (var n in cmdNames) NullifyCommandProp(node, n);

            // clear boolean flags templates might check for
            try
            {
                var t = node.GetType();
                var boolNames = new[] { "CanDelete", "Deletable", "AllowDelete", "IsDeletable", "AllowRemove", "Removable", "CanAdd", "AllowAdd" };
                foreach (var n in boolNames)
                {
                    var pi = t.GetProperty(n, BindingFlags.Instance | BindingFlags.Public | BindingFlags.IgnoreCase);
                    if (pi != null && pi.CanWrite && pi.PropertyType == typeof(bool))
                        pi.SetValue(node, false);
                }
            }
            catch { }

            // recurse children
            if (node.Children != null)
            {
                foreach (var c in node.Children)
                {
                    if (c != null) ApplyFlagsToNode(c);
                }
            }
        }

        // apply to every canonical provided param child under params node
        foreach (var child in paramsNode.Children)
        {
            if (child == null) continue;
            if (string.IsNullOrWhiteSpace(child.Title)) continue;

            if (_providedParamNames.Contains(child.Title))
            {
                ApplyFlagsToNode(child);
            }
        }

        // reorder paramsNode.Children: provided (canonical order) first, then other user params (alphabetical)
        try
        {
            var providedOrder = _providedParamNames.ToList(); // canonical list order
            var providedList = new List<TreeItemVm>();
            var otherList = new List<TreeItemVm>();

            foreach (var child in paramsNode.Children)
            {
                if (child == null) continue;
                if (string.IsNullOrWhiteSpace(child.Title)) { otherList.Add(child); continue; }
                if (_providedParamNames.Contains(child.Title))
                    providedList.Add(child);
                else
                    otherList.Add(child);
            }

            // sort providedList by canonical order (stable)
            providedList.Sort((a, b) =>
            {
                var ai = providedOrder.IndexOf(a.Title ?? "");
                var bi = providedOrder.IndexOf(b.Title ?? "");
                return ai.CompareTo(bi);
            });

            // sort otherList alphabetically by Title
            otherList.Sort((a, b) => string.Compare(a.Title, b.Title, StringComparison.OrdinalIgnoreCase));

            // if current order already matches desired order, no-op
            bool needsReorder = false;
            int idx = 0;
            var desired = providedList.Concat(otherList).ToList();
            if (desired.Count == paramsNode.Children.Count)
            {
                foreach (var node in paramsNode.Children)
                {
                    if (!object.ReferenceEquals(node, desired[idx]))
                    {
                        needsReorder = true;
                        break;
                    }
                    idx++;
                }
            }
            else
            {
                needsReorder = true;
            }

            if (needsReorder)
            {
                paramsNode.Children.Clear();
                foreach (var n in desired) paramsNode.Children.Add(n);
            }
        }
        catch
        {
            // tolerate failures; ordering is a nicety not critical
        }
    }

    // Log reasons why individual targets may not be plotted (best-effort, non-throwing).
    // Outputs concise messages to DebugLog via Log(...).
    private void LogTargetPlotEligibility()
    {
        try
        {
            if (_doc?.Targets == null) return;
            for (int i = 0; i < _doc.Targets.Count; i++)
            {
                var t = _doc.Targets[i];
                if (t == null)
                {
                    Log($"Target[{i}] not plotted: null target entry.");
                    continue;
                }

                // Build a short descriptor to help correlate messages with the UI list
                string desc;
                try
                {
                    var module = string.IsNullOrWhiteSpace(t.Module) ? "(no module)" : t.Module;
                    if (t.Fixture != null)
                        desc = $"module={module} fixture={t.Fixture.Type ?? "(type)"}:{t.Fixture.Index}";
                    else if (t.Translation != null)
                        desc = $"module={module} translation";
                    else
                        desc = $"module={module} (no location)";
                }
                catch { desc = $"target[{i}]"; }

                // If no spatial location at all -> not plotted
                if (t.Translation == null && t.Fixture == null)
                {
                    // However, if the resolver produced a target position for this index, note that.
                    if (_resolvedTargetPositions.TryGetValue(i, out var rt))
                    {
                        Log($"Target[{i}] will be plotted using resolver result: x={rt.x}, y={rt.y} ({desc}).");
                    }
                    else
                    {
                        Log($"Target[{i}] not plotted: no translation or fixture ({desc}).");
                    }
                    continue;
                }

                // If fixture reference present, ensure resolver provided a position for its key
                if (t.Fixture != null)
                {
                    try
                    {
                        var key = $"{t.Fixture.Type}:{t.Fixture.Index}";
                        if (!_resolvedFixturePositions.ContainsKey(key))
                        {
                            Log($"Target[{i}] not plotted: fixture '{key}' missing from resolver output ({desc}). Ensure fixture resolver ran and keys match (type:index).");
                        }
                        // else resolved -> will be plotted (no log)
                    }
                    catch (Exception ex)
                    {
                        Log($"Target[{i}] fixture check failed: {ex.Message}");
                    }
                    continue;
                }

                // Translation present: verify X/Y can be resolved to numeric doubles
                var tr = t.Translation;
                if (tr == null)
                {
                    Log($"Target[{i}] not plotted: translation unexpectedly null ({desc}).");
                    continue;
                }

                if (tr.Position == null || tr.Position.Count < 2)
                {
                    Log($"Target[{i}] not plotted: translation.position missing or too short ({desc}).");
                    continue;
                }

                // Try to resolve the first two coordinates (best-effort, uses _evaluatedLookup)
                if (!TryResolveDoubleFromObject(tr.Position[0], out var x) || !TryResolveDoubleFromObject(tr.Position[1], out var y))
                {
                    // If the resolver produced a numeric position for this target, prefer that and log accordingly
                    if (_resolvedTargetPositions.TryGetValue(i, out var rt2))
                    {
                        Log($"Target[{i}] will be plotted using resolver result: x={rt2.x}, y={rt2.y} ({desc}).");
                        continue;
                    }

                    // Provide hint if the values look like parameter expressions (strings starting with '$') or are non-numeric
                    string hint = "";
                    try
                    {
                        var rawX = tr.Position[0];
                        var rawY = tr.Position[1];
                        if ((rawX is string sx && sx.StartsWith("$")) || (rawY is string sy && sy.StartsWith("$")))
                            hint = " (contains parameter/expression; ensure params evaluated or resolver ran)";
                        else
                            hint = $" (X/Y not numeric or unresolved: [{rawX}, {rawY}]) ";
                    }
                    catch { hint = " (could not analyze X/Y)"; }

                    Log($"Target[{i}] not plotted: translation X/Y could not be resolved{hint} ({desc}).");
                    continue;
                }

                // If we get here, X/Y are numeric and target should be plotted; no log needed.
            }
        }
        catch (Exception ex)
        {
            // Never throw from diagnostic logging
            Log($"LogTargetPlotEligibility failed: {ex.Message}");
        }
    }
}
