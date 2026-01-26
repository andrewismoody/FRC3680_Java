using System.Collections.ObjectModel;
using System.IO;
using System.Text.Json;
using System.Windows.Input;
using Microsoft.Win32;
using AutoJsonBuilder.Models;
using System.Linq;
using AutoJsonBuilder.Helpers; // <-- added
using System.Threading.Tasks; // <-- added near other usings
using System.Windows; // <-- added for Dispatcher
using System.Threading; // <-- add this near other usings
using System.Diagnostics; // <-- added for timing
using OxyPlot;
using OxyPlot.Series;
using OxyPlot.Annotations;
using OxyPlot.Axes;
using System.Runtime.CompilerServices; // add near other usings
using System.Text; // <-- added for StringBuilder / Encoding

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

    // Try to resolve an arbitrary "raw" value into a double.
    // Accepts numbers, boxed numeric types, numeric strings, or parameter name strings that exist in _evaluatedLookup.
    // Returns true and sets out value on success.
    private bool TryResolveDoubleFromObject(object? raw, out double value)
    {
        value = 0.0;
        try
        {
            if (raw is null) return false;

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

            // string: numeric literal or param key
            if (raw is string s)
            {
                if (double.TryParse(s, System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var parsed))
                {
                    value = parsed; return true;
                }
                // look up evaluated params / values
                if (_evaluatedLookup.TryGetValue(s, out var ev))
                {
                    return TryResolveDoubleFromObject(ev, out value);
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

    // No-op ResolveFixturePositions: resolution is performed by the external Java resolver (EvaluateAllExpressions).
    // Keep this method so callers (UpdatePlotModel) can call it safely without duplicating logic in C#.
    private void ResolveFixturePositions()
    {
        // intentionally empty — _resolvedFixturePositions is populated by EvaluateAllExpressions via FixtureResolverInterop.Resolve.
    }

    // Rebuild the PlotModel from current _doc fixtures. Groups fixtures by Type and plots XY points.
    private void UpdatePlotModel()
    {
        try
        {
            // ensure fixture resolution is up-to-date before plotting.
            // Do NOT evaluate expressions here; the Java resolver performs expression evaluation.
            try { ResolveFixturesOnly(); } catch { /* best-effort */ }

            var pm = new PlotModel { Title = "Field fixtures" };

            // Simple pan/zoom axes (X = horizontal, Y = vertical)
            pm.Axes.Add(new LinearAxis { Position = AxisPosition.Bottom, Title = "X" });
            pm.Axes.Add(new LinearAxis { Position = AxisPosition.Left, Title = "Y" });

            // Ensure we have resolved positions (lookup) available as a fast-path (optional)
            ResolveFixturePositions();

            // Group fixtures by type
            var groups = _doc.Fixtures
                .Where(f => f != null)
                .GroupBy(f => f.Type ?? "(none)")
                .ToList();

            var colorIdx = 0;
            var totalFixtures = _doc.Fixtures.Count;
            var plottedCount = 0;

            // collect arrow annotations so we can size them after the PlotModel has axes/transforms computed
            var collectedArrows = new List<ArrowAnnotation>();

            foreach (var g in groups)
            {
                var type = g.Key;
                var color = _palette[colorIdx % _palette.Length];
                colorIdx++;

                // Scatter series for the fixture points
                var scatter = new ScatterSeries
                {
                    MarkerType = MarkerType.Circle,
                    MarkerFill = color,
                    MarkerSize = 6,
                    Title = type
                };

                // Optional arrow annotations for rotation (collect per-group)
                var arrows = new List<ArrowAnnotation>();

                foreach (var f in g)
                {
                    if (TryGetFixturePosition(f, out var fx, out var fy, out var fz, out var frot))
                    {
                        scatter.Points.Add(new ScatterPoint(fx, fy));
                        plottedCount++;
                        if (frot.HasValue)
                        {
                            var theta = frot.Value * Math.PI / 180.0;
                            // Use ~12 inches in meters for arrow length so it matches field units (resolver emits meters)
                            var arrowLengthMeters = 12.0 * 0.0254; // 12 in = 0.3048 m
                            var ex = fx + Math.Cos(theta) * arrowLengthMeters;
                            var ey = fy + Math.Sin(theta) * arrowLengthMeters;

                            var ann = new ArrowAnnotation
                            {
                                StartPoint = new DataPoint(fx, fy),
                                EndPoint = new DataPoint(ex, ey),
                                Color = color,
                                // placeholder head sizes; will be resized after pm.Update(...)
                                HeadLength = 12,
                                HeadWidth = 8
                            };
                            arrows.Add(ann);
                            collectedArrows.Add(ann);
                        }
                    }
                    else
                    {
                        // skip but log minimally for diagnostics
                        Log($"UpdatePlotModel: skipped fixture {f.Type}:{f.Index} (no resolved position)");
                    }
                }

                if (scatter.Points.Count > 0)
                {
                    pm.Series.Add(scatter);
                    // do not add arrows yet; we'll size them after Update
                }
            }

            // Add collected arrows to the model now; sizes will be kept in sync by the Updated handler below.
            foreach (var ann in collectedArrows) pm.Annotations.Add(ann);

            // Recompute arrow head sizes on every PlotModel update so heads scale with zoom/pan.
            pm.Updated += (sender, ev) =>
            {
                try
                {
                    var xAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Bottom) ?? pm.Axes.FirstOrDefault();
                    var yAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Left) ?? pm.Axes.Skip(1).FirstOrDefault() ?? xAxis;
                    var plotArea = pm.PlotArea;
                    if (xAxis == null || yAxis == null || plotArea.Width <= 0 || plotArea.Height <= 0) return;

                    double xRange = xAxis.ActualMaximum - xAxis.ActualMinimum;
                    double yRange = yAxis.ActualMaximum - yAxis.ActualMinimum;
                    if (xRange <= 0 || yRange <= 0) return;

                    double pxPerUnitX = plotArea.Width / xRange;
                    double pxPerUnitY = plotArea.Height / yRange;

                    foreach (var a in pm.Annotations.OfType<ArrowAnnotation>())
                    {
                        try
                        {
                            var dx = a.EndPoint.X - a.StartPoint.X;
                            var dy = a.EndPoint.Y - a.StartPoint.Y;
                            var pixelLen = Math.Sqrt((dx * pxPerUnitX) * (dx * pxPerUnitX) + (dy * pxPerUnitY) * (dy * pxPerUnitY));
                            var newHeadLength = Math.Max(6, pixelLen * 0.35);
                            var newHeadWidth = Math.Max(4, pixelLen * 0.25);
                            // avoid tiny updates that thrash rendering
                            if (Math.Abs(a.HeadLength - newHeadLength) > 0.5 || Math.Abs(a.HeadWidth - newHeadWidth) > 0.5)
                            {
                                a.HeadLength = newHeadLength;
                                a.HeadWidth = newHeadWidth;
                            }
                        }
                        catch { /* per-annotation best-effort */ }
                    }
                }
                catch { /* best-effort */ }
            };

            // Small legend
            pm.IsLegendVisible = true;
            FieldPlotModel = pm;
            Log($"UpdatePlotModel: plotted {plottedCount}/{totalFixtures} fixtures");
        }
        catch (Exception ex)
        {
            Log($"UpdatePlotModel failed: {ex.Message}");
        }
    }

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

    // Resolve fixtures using the Java ResolvedFixtureDump (or other configured interop).
    // Populates _resolvedFixturePositions used exclusively by plotting code.
    private void ResolveFixturesOnly()
    {
        _resolvedFixturePositions.Clear();
        try
        {
            string? outputJson = null;
            // If we have a saved file path, prefer calling the Java tool with that path (use the actual file).
            // Call ResolveFromFile unconditionally when CurrentJsonPath != null so the Java loader sees the exact file.
            if (!string.IsNullOrWhiteSpace(CurrentJsonPath))
            {
                Log($"ResolveFixturesOnly: attempting ResolveFromFile('{CurrentJsonPath}')");
                outputJson = FixtureResolverInterop.ResolveFromFile(CurrentJsonPath);
                if (string.IsNullOrWhiteSpace(outputJson))
                {
                    Log($"ResolveFixturesOnly: ResolveFromFile failed for '{CurrentJsonPath}', falling back to in-memory JSON");
                }
            }

            // Fallback: send in-memory JSON (used for unsaved docs or if ResolveFromFile failed)
            if (string.IsNullOrWhiteSpace(outputJson))
            {
                var docJson = JsonSerializer.Serialize(_doc, JsonOptionsCamel);
                outputJson = FixtureResolverInterop.Resolve(docJson);
            }
            if (string.IsNullOrWhiteSpace(outputJson)) return;

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
                    return "poses — Definitions of named poses (group, location, index, position, action) used by events and targets.";
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

        // moved to PoseHelper
        PoseHelper.EnsurePoseBackingLists(_doc);

        // Ensure modules list exists and seed a sensible default so targets can reference it.
        if (_doc.Modules == null || _doc.Modules.Count == 0)
            _doc.Modules.Clear(); // defensive (if null-handling in model differs)
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
        var json = JsonSerializer.Serialize(_doc, JsonOptionsCamelIndented);
        await File.WriteAllTextAsync(path, json);
        CurrentJsonPath = path;

        // update root label etc.
        await RebuildTreeAsync();

        Log($"SaveToPath: wrote '{path}'.");
    }

    private async Task OpenJsonAsync()
    {
        var dlg = new OpenFileDialog
        {
            Title = "Open auto JSON",
            Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*",
            CheckFileExists = true
        };
        if (dlg.ShowDialog() != true) return;

        // Clear previous debug output when opening a file so the log reflects the new load.
        DebugLog = "";
        // Suppress autosave while we perform the open sequence (rebuilds, helpers, etc).
        Interlocked.Increment(ref _suppressAutoSaveDuringLoad);
        try
        {
            var json = await File.ReadAllTextAsync(dlg.FileName);

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

            // If deserialization produced different params representation, ensure _paramsMap is at least populated from _doc
            if (_paramsMap.Count == 0) UpdateParamsMap();

            CurrentJsonPath = dlg.FileName;
            await RebuildTreeAsync();

            // note: do NOT perform autosave or disk writes while suppression is active.
            Log($"OpenJson: loaded (in-memory) '{dlg.FileName}'.");
        }
        finally
        {
            // always clear suppression so subsequent model-changed callbacks will perform autosave/prompt
            Interlocked.Decrement(ref _suppressAutoSaveDuringLoad);

            // Do NOT perform disk operations automatically after load to avoid intermittent blocking (AV, FS hooks).
            // Instead, apply buffered logs to the in-memory DebugLog in one UI update (cheap).
            try
            {
                ApplyBufferedLogsToUi();
            }
            catch
            {
                // best-effort only
            }
        }
    }

    // When suppression is active the global callback should be a no-op; this check is used by SavePromptOrAutoSaveAsync.
    private bool IsAutosaveSuppressed => Thread.VolatileRead(ref _suppressAutoSaveDuringLoad) > 0;

    private async Task SaveAsJsonAsync()
    {
        var dlg = new SaveFileDialog
        {
            Title = "Save auto JSON",
            Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*",
            FileName = Path.GetFileName(CurrentJsonPath) ?? "auto.json",
            OverwritePrompt = true,
            InitialDirectory = FileHelper.GetDefaultDeployFolder() ?? Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments)
        };
        if (dlg.ShowDialog() != true) return;

        var json = JsonSerializer.Serialize(_doc, JsonOptionsCamelIndented);
        await File.WriteAllTextAsync(dlg.FileName, json);
        CurrentJsonPath = dlg.FileName;

        await RebuildTreeAsync(); // update root label

        Log($"SaveAsJson: wrote '{dlg.FileName}'.");
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

    // NEW: build the entire tree into a root TreeItemVm but do not mutate ObservableCollection.
    private TreeItemVm BuildTreeRoot()
    {
        // ensure evaluated lookup is fresh for this document
        try { EvaluateParamsOnly(); } catch { /* best-effort */ }

        Log("BuildTreeRoot()");
        // local copies / ensure any prep needed
        var rootTitle = Path.GetFileName(CurrentJsonPath) ?? "(unsaved)";
        var root = new TreeItemVm(rootTitle, TreeItemKind.Root) { IsExpanded = true };

        // required scalars
        root.Children.Add(TreeHelper.CreateField("season", _doc, () => _doc.Season, v => _doc.Season = v));
        root.Children.Add(TreeHelper.CreateField("version", _doc, () => _doc.Version, v => _doc.Version = v));
        root.Children.Add(TreeHelper.CreateField("description", _doc, () => _doc.Description, v => _doc.Description = v));

        // sections (these Create* helpers create TreeItemVm instances, which is the bulk of the work)
        root.Children.Add(BuildParamsSection());
        // add modules section (editable list of module labels)
        root.Children.Add(BuildStringListSection("modules", _doc.Modules));
        root.Children.Add(BuildStringListSection("groups", _doc.Groups));
        root.Children.Add(BuildStringListSection("travelGroups", _doc.TravelGroups, _doc.Groups));
        root.Children.Add(BuildStringListSection("locations", _doc.Locations));
        root.Children.Add(BuildStringListSection("positions", _doc.Positions));
        root.Children.Add(BuildStringListSection("actions", _doc.Actions));
        root.Children.Add(BuildPosesSection());
        root.Children.Add(BuildEventsSection());
        root.Children.Add(BuildFixturesSection());
        root.Children.Add(BuildTargetsSection());
        root.Children.Add(BuildSequencesSection());

        // Make sure options are computed for nodes that rely on ParamKeys etc.
        // (RefreshAllOptions will be invoked after the root is attached to the ObservableCollection.)
        return root;
    }

    // RebuildTree now simply constructs and installs the tree synchronously (kept for compatibility).
    private void RebuildTree()
    {
        var sw = Stopwatch.StartNew();
        var prev = TreeHelper.GetViewState(TreeRootItems);

        var root = BuildTreeRoot();
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

    private TreeItemVm BuildParamsSection()
    {
        var sec = new TreeItemVm("params", TreeItemKind.Section) { HasAdd = true };
        sec.AddCommand = new RelayCommand(() =>
        {
            var key = $"param{_doc.Params.Count + 1}";
            // new ParamValue wrapper for a numeric 0 default
            _doc.Params[key] = new AutoJsonBuilder.Models.ParamValue(0d);
            _ = RebuildTreeAsync(); // command handler stays sync, rebuild asynchronously
            TreeHelper.TriggerModelChangedImmediate();
            TreeHelper.FocusNewUnder(TreeRootItems, "params", key);
        });

        foreach (var kvp in _doc.Params.OrderBy(k => k.Key))
        {
            var raw = kvp.Value?.Raw;

            // If param is an array (preserved as List<object>), create an object node with one child per element.
            if (raw is IList<object> list)
            {
                var originalKey = kvp.Key;
                var originalVal = kvp.Value;
                // Make array-backed params editable like scalars: allow renaming the param key via the caption
                var node = new TreeItemVm(kvp.Key, TreeItemKind.Object)
                {
                    HasRemove = true,
                    Model = kvp,
                    IsCaptionEditable = true
                };
                // Commit caption edits with the same validation/rename flow used for scalar params
                node.PropertyChanged += async (_, e) =>
                {
                    if (e.PropertyName != nameof(TreeItemVm.Title)) return;
                    var newKey = node.Title?.Trim() ?? "";
                    if (string.IsNullOrWhiteSpace(newKey) || newKey == originalKey)
                    {
                        node.Title = originalKey;
                        return;
                    }
                    if (_doc.Params.ContainsKey(newKey))
                    {
                        node.Title = originalKey; // revert on collision
                        return;
                    }
                    await RenameParamAsync(originalKey, newKey);
                };
                node.RemoveCommand = new RelayCommand(() => { _doc.Params.Remove(originalKey); _ = RebuildTreeAsync(); TreeHelper.TriggerModelChangedImmediate(); });

                // Add command to append a new element, only if < 3 elements
                node.HasAdd = list.Count < 3;
                node.AddCommand = new RelayCommand(() =>
                {
                    if (list.Count >= 3) return;
                    list.Add(0d);
                    _ = RebuildTreeAsync();
                    TreeHelper.TriggerModelChangedImmediate();
                    TreeHelper.FocusNewUnder(TreeRootItems, "params", originalKey);
                });

                for (var i = 0; i < list.Count; i++)
                {
                    var index = i;
                    // create per-element editor using TreeHelper so it gets the same NumberOrParam behavior/hints
                    var child = TreeHelper.CreateNumberOrParamField(
                        TreeHelper.GetArrayIndexLabel(index),
                        kvp.Value!,                     // model (ParamValue)
                        () => list[index],              // getter returns element
                        v => { list[index] = v; },      // setter updates element in-place
                        _paramsMap,                     // params map for previews/options
                        captionEditable: true           // allow editing element title if desired (keeps parity)
                    );

                    // Param array elements should be plain text editors (no combobox of param names)
                    child.Editor = TreeEditorKind.Text;
                    child.Options.Clear();

                    // Only expose a remove icon on the last element (when array length is 2 or 3).
                    // If removing the last element when the array has 2 elements, convert the param back to a scalar.
                    if (list.Count >= 2 && index == list.Count - 1)
                    {
                        child.HasRemove = true;
                        var idx = index; // capture for lambda
                        child.RemoveCommand = new RelayCommand(() =>
                        {
                            // refresh local references in case of closure
                            var key = originalKey;
                            var valWrapper = originalVal;

                            if (list.Count == 2)
                            {
                                // convert to scalar: keep first element as the scalar value
                                var first = list.Count > 0 ? list[0] : 0d;
                                valWrapper.Raw = first!;
                                _doc.Params[key] = valWrapper!;
                            }
                            else
                            {
                                // list.Count == 3 (or >2) -> remove the last element
                                if (idx >= 0 && idx < list.Count) list.RemoveAt(idx);
                            }

                            UpdateParamsMap();
                            _ = RebuildTreeAsync();
                            TreeHelper.TriggerModelChangedImmediate();
                            TreeHelper.FocusNewUnder(TreeRootItems, "params", key);
                        });
                    }

                    node.Children.Add(child);
                }

                sec.Children.Add(node);
            }
            else
            {
                var originalKey = kvp.Key;
                var originalVal = kvp.Value;

                // Represent scalar param as an OBJECT node so its Title is editable in the TreeView.
                var node = new TreeItemVm(originalKey, TreeItemKind.Object)
                {
                    HasRemove = true,
                    Model = kvp
                };
                node.RemoveCommand = new RelayCommand(() => { _doc.Params.Remove(originalKey); _ = RebuildTreeAsync(); TreeHelper.TriggerModelChangedImmediate(); });

                // Allow renaming the param key via editable caption on the parent object node
                node.IsCaptionEditable = true;
                node.PropertyChanged += async (_, e) =>
                {
                    if (e.PropertyName != nameof(TreeItemVm.Title)) return;
                    var newKey = node.Title?.Trim() ?? "";
                    if (string.IsNullOrWhiteSpace(newKey) || newKey == originalKey)
                    {
                        node.Title = originalKey;
                        return;
                    }
                    if (_doc.Params.ContainsKey(newKey))
                    {
                        node.Title = originalKey; // revert on collision
                        return;
                    }
                    await RenameParamAsync(originalKey, newKey);
                };

                // Add button converts scalar -> array (initial element is current scalar), only allowed if conversion makes sense
                node.HasAdd = true;
                node.AddCommand = new RelayCommand(() =>
                {
                    // convert scalar Raw to list: keep current scalar as first element and add one extra default element
                    var rawVal = originalVal?.Raw;
                    if (rawVal is IList<object>) return; // already an array
                    var newList = new List<object>();
                    newList.Add(rawVal ?? 0d); // preserve existing value as first element
                    newList.Add(0d);           // add an extra element so UI shows a change
                    originalVal.Raw = newList;
                    _doc.Params[originalKey] = originalVal!;
                    UpdateParamsMap();
                    _ = RebuildTreeAsync();
                    TreeHelper.TriggerModelChangedImmediate();
                    TreeHelper.FocusNewUnder(TreeRootItems, "params", originalKey);
                });

                // Child "value" field holds the actual scalar value/expression (plain text editor)
                var child = TreeHelper.CreateNumberOrParamField(
                    "value",
                    kvp.Value!,                      // model (ParamValue)
                    () => kvp.Value?.Raw,            // getter: raw value
                    v => { kvp.Value.Raw = v; },     // setter: update raw
                    _paramsMap,                      // params map for previews/options
                    captionEditable: false
                );
                child.Editor = TreeEditorKind.Text;
                child.Options.Clear();
                child.Model = kvp;

                node.Children.Add(child);
                sec.Children.Add(node);
            }
        }

        // Refresh ParamKeys observable so other UI pieces can use current names
        ParamKeys.Clear();
        foreach (var k in _doc.Params.Keys.OrderBy(x => x)) ParamKeys.Add(k);

        return sec;
    }

    private TreeItemVm BuildStringListSection(string name, IList<string> list, IList<string>? optionsSource = null)
    {
        var sec = new TreeItemVm(name, TreeItemKind.Section) { HasAdd = true };

        // Add behavior: if optionsSource supplied, new entry defaults to first option (or empty)
        sec.AddCommand = new RelayCommand(() =>
        {
            var baseName = GetSingularName(name);
            var newVal = optionsSource != null && optionsSource.Count > 0 ? optionsSource[0] : $"{baseName}{list.Count + 1}";
            list.Add(newVal);
            _ = RebuildTreeAsync();           // guarantees dropdowns are rebuilt with fresh Options
            TreeHelper.TriggerModelChangedImmediate();
            TreeHelper.FocusNewUnder(TreeRootItems, name, list.Last());
        });

        for (var i = 0; i < list.Count; i++)
        {
            var index = i;
            if (optionsSource != null)
            {
                // Render as a dropdown-only field (no caption). Use empty label and disable caption editing.
                var field = TreeHelper.CreateListField("", list, () => list[index], v => list[index] = v, optionsSource);
                field.IsCaptionEditable = false;
                field.HasRemove = true;
                field.Model = list;
                field.RemoveCommand = new RelayCommand(() =>
                {
                    if (index >= 0 && index < list.Count) list.RemoveAt(index);
                    _ = RebuildTreeAsync();
                    TreeHelper.TriggerModelChangedImmediate();
                });
                sec.Children.Add(field);
            }
            else
            {
                // Legacy behavior: editable caption object node
                var node = new TreeItemVm(list[index], TreeItemKind.Object)
                {
                    IsCaptionEditable = true,
                    Editor = TreeEditorKind.None,
                    HasRemove = true
                };

                node.PropertyChanged += (_, e) =>
                {
                    if (e.PropertyName != nameof(TreeItemVm.Title)) return;
                    var newValue = node.Title?.Trim() ?? "";
                    if (string.IsNullOrWhiteSpace(newValue)) return;

                    list[index] = newValue;
                    _ = RebuildTreeAsync();       // guarantees pose dropdowns are rebuilt with fresh Options
                };

                node.RemoveCommand = new RelayCommand(() =>
                {
                    list.RemoveAt(index);
                    _ = RebuildTreeAsync();       // guarantees pose dropdowns are rebuilt with fresh Options
                    TreeHelper.TriggerModelChangedImmediate();
                });

                sec.Children.Add(node);
            }
        }

        return sec;
    }

    // Simple singularization for generated names: "categories" -> "category", "groups" -> "group"
    private static string GetSingularName(string plural)
    {
        if (string.IsNullOrWhiteSpace(plural)) return plural ?? "";
        if (plural.EndsWith("ies", StringComparison.OrdinalIgnoreCase) && plural.Length > 3)
            return plural.Substring(0, plural.Length - 3) + "y";
        if (plural.EndsWith("s", StringComparison.OrdinalIgnoreCase) && plural.Length > 1)
            return plural.Substring(0, plural.Length - 1);
        return plural;
    }

    private TreeItemVm BuildPosesSection()
    {
        Log($"BuildPosesSection: groups={_doc.Groups.Count}, locations={_doc.Locations.Count}, positions={_doc.Positions.Count}, actions={_doc.Actions.Count}");

        var sec = new TreeItemVm("poses", TreeItemKind.Section) { HasAdd = true };
        sec.AddCommand = new RelayCommand(() =>
        {
            var newPose = new PoseModel
            {
                Group = _doc.Groups.FirstOrDefault() ?? "",
                Location = _doc.Locations.FirstOrDefault() ?? "",
                Index = 0,
                Position = _doc.Positions.FirstOrDefault() ?? "",
                Action = _doc.Actions.FirstOrDefault() ?? ""
            };

            _doc.Poses.Add(newPose);

            // ensure stable unique names after adding (moved)
            PoseHelper.EnsurePoseNames(_doc);

            // Rebuild UI and notify model-changed so autosave/prompt runs for this user action.
            _ = RebuildTreeAsync();
            TreeHelper.TriggerModelChangedImmediate();
            TreeHelper.FocusNewUnder(TreeRootItems, "poses");
        });

        foreach (var p in _doc.Poses)
        {
            var originalTitle = p.Name ?? $"pose[{p.Group},{p.Location},{p.Index}]";
			var node = new TreeItemVm(originalTitle, TreeItemKind.Object)
			{
				HasRemove = true,
				RemoveCommand = new RelayCommand(() => { _doc.Poses.Remove(p); PoseHelper.EnsurePoseNames(_doc); _ = RebuildTreeAsync(); TreeHelper.TriggerModelChangedImmediate(); }),
				Model = p,
				// allow editing the pose label; do NOT react to Title changes here.
				IsCaptionEditable = true
			};
			// NOTE: do not handle Title PropertyChanged here. The view should call CommitPoseCaptionAsync(node)
			// from the caption TextBox LostFocus handler so renames only commit on LostFocus.

            // fixed label: use "group" not "group_TEST"
            node.Children.Add(TreeHelper.CreatePoseDropdown("group", p, () => p.Group, v => p.Group = v, _doc.Groups));
            node.Children.Add(TreeHelper.CreatePoseDropdown("location", p, () => p.Location, v => p.Location = v, _doc.Locations));
            node.Children.Add(TreeHelper.CreateIntField("index", p, () => p.Index, v => p.Index = v));
            node.Children.Add(TreeHelper.CreatePoseDropdown("position", p, () => p.Position, v => p.Position = v, _doc.Positions));
            node.Children.Add(TreeHelper.CreatePoseDropdown("action", p, () => p.Action, v => p.Action = v, _doc.Actions));

            sec.Children.Add(node);
        }

        return sec;
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

    private TreeItemVm BuildEventsSection()
    {
        var sec = new TreeItemVm("events", TreeItemKind.Section) { HasAdd = true };
        sec.AddCommand = new RelayCommand(() =>
        {
            var poseName = _doc.Poses.FirstOrDefault()?.Name ?? "pose1";
            var ev = new EventModel
            {
                Name = $"event{_doc.Events.Count + 1}",
                Type = "await",
                Parallel = false,
                Pose = poseName,
                TriggerType = "none",
                TriggerValue = false,
                TriggerModule = null,
                TriggerInvert = false
            };
            _doc.Events.Add(ev);
            _ = RebuildTreeAsync();
            TreeHelper.TriggerModelChangedImmediate();
            TreeHelper.FocusNewUnder(TreeRootItems, "events");
        });

        foreach (var ev in _doc.Events)
        {
            // Make event caption editable so users can rename events directly from the tree.
            var originalTitle = ev.Name;
            var node = new TreeItemVm(ev.Name, TreeItemKind.Object)
            {
                HasRemove = true,
                RemoveCommand = new RelayCommand(() => { _doc.Events.Remove(ev); _ = RebuildTreeAsync(); }),
                Model = ev,
                IsCaptionEditable = true
            };
            // Commit caption edits: validate, avoid collisions, and rename via helper
            node.PropertyChanged += async (_, e) =>
            {
                if (e.PropertyName != nameof(TreeItemVm.Title)) return;
                var newTitle = node.Title?.Trim() ?? "";
                var oldName = originalTitle ?? "";
                if (string.IsNullOrWhiteSpace(newTitle))
                {
                    node.Title = oldName; // revert invalid
                    return;
                }
                if (string.Equals(newTitle, oldName, StringComparison.OrdinalIgnoreCase)) return;
                if (_doc.Events.Any(x => string.Equals(x.Name, newTitle, StringComparison.OrdinalIgnoreCase)))
                {
                    node.Title = oldName; // collision -> revert
                    return;
                }
                await RenameEventAsync(oldName, newTitle);
            };

            // type (await|time)
            var type = new TreeItemVm("type", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.Type, Model = ev };
            type.Options.Add("await"); type.Options.Add("time");
            type.PropertyChanged += (_, e) =>
            {
                if (e.PropertyName == nameof(TreeItemVm.Value))
                {
                    ev.Type = type.Value?.ToString() ?? "await";
                    TreeHelper.TriggerModelChangedImmediate();
                }
            };
            node.Children.Add(type);

            // parallel (dropdown true/false)
            var parallel = new TreeItemVm("parallel", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.Parallel ? "true" : "false", Model = ev };
            parallel.Options.Add("true"); parallel.Options.Add("false");
            parallel.PropertyChanged += (_, e) =>
            {
                if (e.PropertyName == nameof(TreeItemVm.Value))
                {
                    var s = parallel.Value?.ToString();
                    ev.Parallel = string.Equals(s, "true", StringComparison.OrdinalIgnoreCase);
                    TreeHelper.TriggerModelChangedImmediate();
                }
            };
            node.Children.Add(parallel);

            // pose (dropdown of existing pose names). Include an empty option to allow "no pose" selection.
            var poseField = new TreeItemVm("pose", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.Pose ?? "", Model = ev };
            poseField.Options.Clear();
            // first option is empty meaning "no pose"
            poseField.Options.Add("");
            foreach (var pName in _doc.Poses.Select(p => p.Name ?? ""))
                poseField.Options.Add(pName);
            poseField.PropertyChanged += (_, e) =>
            {
                if (e.PropertyName == nameof(TreeItemVm.Value))
                {
                    var sel = poseField.Value?.ToString() ?? "";
                    ev.Pose = string.IsNullOrWhiteSpace(sel) ? null : sel;
                    TreeHelper.TriggerModelChangedImmediate();
                }
            };
            node.Children.Add(poseField);

            // milliseconds
            var ms = new TreeItemVm("milliseconds", TreeItemKind.Field) { Editor = TreeEditorKind.NumberOrParam, Value = ev.Milliseconds ?? 0, Model = ev };
            ms.PropertyChanged += (_, e) =>
            {
                if (e.PropertyName == nameof(TreeItemVm.Value))
                {
                    ev.Milliseconds = Convert.ToInt32(ms.Value ?? 0);
                    TreeHelper.TriggerModelChangedImmediate();
                }
            };
            node.Children.Add(ms);

            // triggerType (enum: boolean | none)
            var tt = new TreeItemVm("triggerType", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.TriggerType ?? "none", Model = ev };
            tt.Options.Add("boolean"); tt.Options.Add("none");
            tt.PropertyChanged += (_, e) =>
            {
                if (e.PropertyName == nameof(TreeItemVm.Value))
                {
                    ev.TriggerType = tt.Value?.ToString();
                    TreeHelper.TriggerModelChangedImmediate();
                }
            };
            node.Children.Add(tt);

            // triggerValue (dropdown true/false)
            var tv = new TreeItemVm("triggerValue", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.TriggerValue, Model = ev };
            tv.Options.Add("true"); tv.Options.Add("false");
            tv.PropertyChanged += (_, e) =>
            {
                if (e.PropertyName == nameof(TreeItemVm.Value))
                {
                    var s = tv.Value?.ToString();
                    ev.TriggerValue = string.Equals(s, "true", StringComparison.OrdinalIgnoreCase);
                    TreeHelper.TriggerModelChangedImmediate();
                }
            };
            node.Children.Add(tv);

            // triggerModule (dropdown of modules; first option is empty => "no module")
            var tm = new TreeItemVm("triggerModule", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.TriggerModule ?? "", Model = ev };
            tm.Options.Clear();
            tm.Options.Add(""); // empty = no module
            foreach (var m in _doc.Modules) tm.Options.Add(m ?? "");
            tm.PropertyChanged += (_, e) =>
            {
                if (e.PropertyName == nameof(TreeItemVm.Value))
                {
                    var sel = tm.Value?.ToString() ?? "";
                    ev.TriggerModule = string.IsNullOrWhiteSpace(sel) ? null : sel;
                    TreeHelper.TriggerModelChangedImmediate();
                }
            };
            node.Children.Add(tm);

            // triggerInvert (dropdown true/false)
            var ti = new TreeItemVm("triggerInvert", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.TriggerInvert ? "true" : "false", Model = ev };
            ti.Options.Add("true"); ti.Options.Add("false");
            ti.PropertyChanged += (_, e) =>
            {
                if (e.PropertyName == nameof(TreeItemVm.Value))
                {
                    var s = ti.Value?.ToString();
                    ev.TriggerInvert = string.Equals(s, "true", StringComparison.OrdinalIgnoreCase);
                    TreeHelper.TriggerModelChangedImmediate();
                }
            };
            node.Children.Add(ti);

            sec.Children.Add(node);
        }
        return sec;
    }

    private TreeItemVm BuildFixturesSection()
    {
        var sec = new TreeItemVm("fixtures", TreeItemKind.Section) { HasAdd = true };
        sec.AddCommand = new RelayCommand(() =>
        {
            var f = new FixtureSchemaModel { Type = "fixture", Index = _doc.Fixtures.Count };
            _doc.Fixtures.Add(f);
            AddFixtureVm(sec, f);
            // Notify model-changed so autosave/prompt will run for this user action.
            TreeHelper.TriggerModelChangedImmediate();
        });

        foreach (var f in _doc.Fixtures)
            AddFixtureVm(sec, f); // builds vms (also selects last; you can remove that behavior if undesired)

        return sec;
    }

    private TreeItemVm BuildTargetsSection()
    {
        var sec = new TreeItemVm("targets", TreeItemKind.Section) { HasAdd = true };
        sec.AddCommand = new RelayCommand(() =>
        {
            var t = new TargetSchemaModel { Module = $"module{_doc.Targets.Count + 1}" };
            _doc.Targets.Add(t);
            AddTargetVm(sec, t);
            // Notify model-changed so autosave/prompt will run for this user action.
            TreeHelper.TriggerModelChangedImmediate();
        });

        foreach (var t in _doc.Targets)
            AddTargetVm(sec, t);

        return sec;
    }

    private TreeItemVm BuildSequencesSection()
    {
        var sec = new TreeItemVm("sequences", TreeItemKind.Section) { HasAdd = true };
        sec.AddCommand = new RelayCommand(() =>
        {
            _doc.Sequences.Add(new SequenceModel {
                Name = $"seq{_doc.Sequences.Count + 1}",
                Events = new() { _doc.Events.FirstOrDefault()?.Name ?? "" },
                Start1 = new(), Start2 = new(), Start3 = new()
            });
            _ = RebuildTreeAsync();
            TreeHelper.TriggerModelChangedImmediate();
            TreeHelper.FocusNewUnder(TreeRootItems, "sequences");
        });

        foreach (var s in _doc.Sequences)
        {
            var originalName = s.Name;
            var node = new TreeItemVm(s.Name, TreeItemKind.Object)
            {
                HasRemove = true,
                RemoveCommand = new RelayCommand(() => { _doc.Sequences.Remove(s); _ = RebuildTreeAsync(); TreeHelper.TriggerModelChangedImmediate(); }),
                Model = s,
                IsCaptionEditable = true
            };
            // Caption rename handler (existing rename flow)
            node.PropertyChanged += async (_, e) =>
            {
                if (e.PropertyName != nameof(TreeItemVm.Title)) return;
                var newName = node.Title?.Trim() ?? "";
                var oldName = originalName ?? "";
                if (string.IsNullOrWhiteSpace(newName))
                {
                    node.Title = oldName; // revert invalid
                    return;
                }
                if (string.Equals(newName, oldName, StringComparison.OrdinalIgnoreCase)) return;
                if (_doc.Sequences.Any(x => string.Equals(x.Name, newName, StringComparison.OrdinalIgnoreCase)))
                {
                    node.Title = oldName; // collision -> revert
                    return;
                }
                await RenameSequenceAsync(oldName, newName);
            };

            // Ensure arrays exist so UI code can safely enumerate / modify
            s.Events ??= new System.Collections.Generic.List<string>();
            s.Start1 ??= new System.Collections.Generic.List<string>();
            s.Start2 ??= new System.Collections.Generic.List<string>();
            s.Start3 ??= new System.Collections.Generic.List<string>();

            // events list (one child node named "events" per schema) — options come from current event names
            var eventOptions = _doc.Events.Select(ev => ev.Name ?? "").Where(n => !string.IsNullOrWhiteSpace(n)).ToList();

            var eventsNode = new TreeItemVm("events", TreeItemKind.Object)
            {
                Model = s.Events,
                HasAdd = true
            };
            eventsNode.AddCommand = new RelayCommand(() =>
            {
                var defaultEvent = eventOptions.FirstOrDefault() ?? "";
                s.Events.Add(defaultEvent);
                _ = RebuildTreeAsync();
                TreeHelper.TriggerModelChangedImmediate();
                TreeHelper.FocusNewUnder(TreeRootItems, "sequences", s.Name);
            });
            for (int i = 0; i < s.Events.Count; i++)
            {
                var idx = i;
                var field = TreeHelper.CreateListField("", s.Events, () => s.Events[idx], v => s.Events[idx] = v, eventOptions);
                field.IsCaptionEditable = false;
                field.HasRemove = true;
                field.Model = s.Events;
                field.RemoveCommand = new RelayCommand(() =>
                {
                    if (idx >= 0 && idx < s.Events.Count) s.Events.RemoveAt(idx);
                    _ = RebuildTreeAsync();
                    TreeHelper.TriggerModelChangedImmediate();
                });
                eventsNode.Children.Add(field);
            }
            node.Children.Add(eventsNode);

            // start1 / start2 / start3 arrays: same pattern as events
            void AddStartArrayNode(string title, System.Collections.Generic.List<string> list)
            {
                var arrNode = new TreeItemVm(title, TreeItemKind.Object)
                {
                    Model = list,
                    HasAdd = true
                };
                arrNode.AddCommand = new RelayCommand(() =>
                {
                    var defaultEvent = eventOptions.FirstOrDefault() ?? "";
                    list.Add(defaultEvent);
                    _ = RebuildTreeAsync();
                    TreeHelper.TriggerModelChangedImmediate();
                    TreeHelper.FocusNewUnder(TreeRootItems, "sequences", s.Name);
                });
                for (int i = 0; i < list.Count; i++)
                {
                    var idx = i;
                    var field = TreeHelper.CreateListField("", list, () => list[idx], v => list[idx] = v, eventOptions);
                    field.IsCaptionEditable = false;
                    field.HasRemove = true;
                    field.Model = list;
                    field.RemoveCommand = new RelayCommand(() =>
                    {
                        if (idx >= 0 && idx < list.Count) list.RemoveAt(idx);
                        _ = RebuildTreeAsync();
                        TreeHelper.TriggerModelChangedImmediate();
                    });
                    arrNode.Children.Add(field);
                }
                node.Children.Add(arrNode);
            }

            AddStartArrayNode("start1", s.Start1);
            AddStartArrayNode("start2", s.Start2);
            AddStartArrayNode("start3", s.Start3);

            sec.Children.Add(node);
        }
        return sec;
    }

    private void AddFixtureVm(TreeItemVm fixturesSection, FixtureSchemaModel f)
    {
        var node = new TreeItemVm($"{f.Type}:{f.Index}", TreeItemKind.Object)
        {
            // fixture captions should not be editable
            IsCaptionEditable = false,
            Model = f,
            HasRemove = true
        };
        node.RemoveCommand = new RelayCommand(() =>
        {
            _doc.Fixtures.Remove(f);
            fixturesSection.Children.Remove(node);
            TreeHelper.TriggerModelChangedImmediate();
        });

        // Editable header fields (type + index) — fixture definitions are direct editable entries.
        node.Children.Add(TreeHelper.CreateTextField("type", f, () => f.Type, v => f.Type = v));
        node.Children.Add(TreeHelper.CreateIntField("index", f, () => f.Index, v => f.Index = v));

        // Build a provider of existing fixture "type:index" strings for fixture refs inside derivedFrom editors.
        // Exclude the fixture being edited (f) to avoid creating a direct circular reference.
        var fixtureOptionsProvider = new Func<IEnumerable<string>>(() =>
            _doc.Fixtures
                .Where(x => !object.ReferenceEquals(x, f))
                .Select(x => $"{x.Type}:{x.Index}")
                .Distinct());

        // Source selector: choose whether this fixture uses translation or derivedFrom
        var source = new TreeItemVm("source", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Model = f };
        source.Options.Add("translation");
        source.Options.Add("derivedFrom");
        source.Value = f.DerivedFrom != null ? "derivedFrom" : "translation";
        node.Children.Add(source);

        // helper to add the active child (translation or derivedFrom)
        void AddActiveChild()
        {
            // remove any existing translation/derivedFrom child
            var existing = node.Children.FirstOrDefault(c => c.Title == "translation" || c.Title == "derivedFrom");
            if (existing != null) node.Children.Remove(existing);

            if (string.Equals(source.Value?.ToString(), "derivedFrom", StringComparison.OrdinalIgnoreCase))
            {
                // ensure derivedFrom model exists
                if (f.DerivedFrom == null)
                    f.DerivedFrom = new DerivedFromModel();
                // add derivedFrom editor (recursive) and pass fixtureOptionsProvider so nested fixture refs become dropdowns
                node.Children.Add(TreeHelper.CreateDerivedFromNode(f.DerivedFrom!, ParamKeys, fixtureOptionsProvider));
                // remove translation to satisfy oneOf
                f.Translation = null;
            }
            else
            {
                // ensure translation model exists
                f.Translation ??= new TranslationModel { Position = new() { 0d, 0d, 0d }, PositionUnits = "inches", RotationUnits = "degrees" };
                node.Children.Add(TreeHelper.CreateTranslationNode(f.Translation!, ParamKeys));
                // remove derivedFrom to satisfy oneOf
                f.DerivedFrom = null;
            }
        }

        // initial child build
        AddActiveChild();

        // when user switches source, swap model children and persist
        source.PropertyChanged += async (_, e) =>
        {
            if (e.PropertyName != nameof(TreeItemVm.Value)) return;
            AddActiveChild();
            TreeHelper.TriggerModelChangedImmediate();
            await RebuildTreeAsync();
        };

        fixturesSection.Children.Add(node);
        fixturesSection.IsExpanded = true;
        node.IsSelected = true;
    }

    private void AddTargetVm(TreeItemVm targetsSection, TargetSchemaModel t)
    {
        // Target caption should NOT be an editable textbox; compute a descriptive label instead.
        var node = new TreeItemVm("", TreeItemKind.Object)
        {
            IsCaptionEditable = false,
            Model = t,
            HasRemove = true
        };
        node.RemoveCommand = new RelayCommand(() =>
        {
            _doc.Targets.Remove(t);
            targetsSection.Children.Remove(node);
            TreeHelper.TriggerModelChangedImmediate();
        });

        // Editable header fields (keep references so we can update the non-editable caption when values change)
        var moduleField = TreeHelper.CreateListField("module", t, () => t.Module ?? "", v => t.Module = string.IsNullOrWhiteSpace(v) ? null : v, _doc.Modules);
        var groupField = TreeHelper.CreatePoseDropdown("group", t, () => t.Group ?? "", v => t.Group = v, _doc.Groups);
        var locationField = TreeHelper.CreatePoseDropdown("location", t, () => t.Location ?? "", v => t.Location = v, _doc.Locations);
        var indexField = TreeHelper.CreateIntField("index", t, () => t.Index ?? -1, v => t.Index = v);
        var positionField = TreeHelper.CreatePoseDropdown("position", t, () => t.Position ?? "", v => t.Position = v, _doc.Positions);
        var actionField = TreeHelper.CreatePoseDropdown("action", t, () => t.Action ?? "", v => t.Action = v, _doc.Actions);

        node.Children.Add(moduleField);
        node.Children.Add(groupField);
        node.Children.Add(locationField);
        node.Children.Add(indexField);
        node.Children.Add(positionField);
        node.Children.Add(actionField);

        // fixture options provider (list of "type:index") for fixture refs inside targets
        var fixtureOptionsProvider = new Func<IEnumerable<string>>(() =>
            _doc.Fixtures.Select(x => $"{x.Type}:{x.Index}").Distinct());

        // kind selector: translation | fixture
        var kindField = new TreeItemVm("kind", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Model = t };
        kindField.Options.Add("translation"); kindField.Options.Add("fixture");
        kindField.Value = t.Fixture != null ? "fixture" : "translation";
        node.Children.Add(kindField);

        // helper to add active child based on kind
        void AddActiveChild()
        {
            var existing = node.Children.FirstOrDefault(c => c.Title == "translation" || c.Title == "fixture");
            if (existing != null) node.Children.Remove(existing);

            if (string.Equals(kindField.Value?.ToString(), "fixture", StringComparison.OrdinalIgnoreCase))
            {
                // ensure fixture ref exists on model
                t.Fixture ??= new FixtureRefModel();
                // add fixture-ref editor (dropdown of known fixtures)
                node.Children.Add(TreeHelper.CreateFixtureRefNode("fixture", t.Fixture,
                    () => t.Fixture?.Type ?? "",
                    v => t.Fixture!.Type = v,
                    () => t.Fixture?.Index ?? 0,
                    v => t.Fixture!.Index = v,
                    fixtureOptionsProvider));
                // clear translation to respect oneOf
                t.Translation = null;
            }
            else
            {
                // ensure translation exists
                t.Translation ??= new TranslationModel { Position = new() { 0d, 0d, 0d }, PositionUnits = "inches", RotationUnits = "degrees" };
                node.Children.Add(TreeHelper.CreateTranslationNode(t.Translation, ParamKeys));
                // clear fixture ref to respect oneOf
                t.Fixture = null;
            }
        }

        // initial child
        AddActiveChild();

        // Helper: compute a concise descriptive title for the target (module + pose or fixture summary)
        void UpdateTitle()
        {
            string modulePart = string.IsNullOrWhiteSpace(t.Module) ? "(no-module)" : t.Module;
            string body;
            if (t.Fixture != null)
            {
                var ft = t.Fixture.Type ?? "(type)";
                var fi = t.Fixture.Index;
                body = $"fixture {ft}:{fi}";
            }
            else
            {
                var g = string.IsNullOrWhiteSpace(t.Group) ? "?" : t.Group;
                var loc = string.IsNullOrWhiteSpace(t.Location) ? "?" : t.Location;
                var idx = t.Index.HasValue ? t.Index.Value.ToString() : "?";
                var pos = string.IsNullOrWhiteSpace(t.Position) ? "?" : t.Position;
                var act = string.IsNullOrWhiteSpace(t.Action) ? "?" : t.Action;
                body = $"{g}/{loc}[{idx}] {pos} {act}";
            }
            node.Title = $"{modulePart} · {body}";
        }

        // Wire up children so updates refresh the computed title.
        moduleField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
        groupField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
        locationField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
        indexField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
        positionField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
        actionField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
        // kind/fixture changes also affect label
        kindField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };

        // initial computed title
        UpdateTitle();

        // when user switches kind, swap child and persist
        kindField.PropertyChanged += async (_, e) =>
        {
            if (e.PropertyName != nameof(TreeItemVm.Value)) return;
            AddActiveChild();
            TreeHelper.TriggerModelChangedImmediate();
            await RebuildTreeAsync();
        };

        targetsSection.Children.Add(node);
        targetsSection.IsExpanded = true;
        node.IsSelected = true;
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
            var json = JsonSerializer.Serialize(_doc, JsonOptionsCamelIndented);
            await File.WriteAllTextAsync(path, json);
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

}
