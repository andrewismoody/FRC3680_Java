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

namespace AutoJsonBuilder;

public sealed class MainWindowViewModel : NotifyBase
{
    // SINGLE FLOW: schema-aligned document only
    private AutoDefinitionModel _doc = new();

    public ObservableCollection<TreeItemVm> TreeRootItems { get; } = new();
    public ObservableCollection<string> ParamKeys { get; } = new();

    private string _selectionDetails = "";
    public string SelectionDetails { get => _selectionDetails; set => Set(ref _selectionDetails, value); }

    private string? _currentJsonPath;
    public string? CurrentJsonPath { get => _currentJsonPath; set => Set(ref _currentJsonPath, value); }

    public ICommand NewCommand { get; }
    public ICommand OpenCommand { get; }
    public ICommand SaveAsCommand { get; }

    private string _debugLog = "";
    public string DebugLog { get => _debugLog; set => Set(ref _debugLog, value); }

    // params map used to resolve expressions for calculated display values
    private IDictionary<string, object> _paramsMap = new Dictionary<string, object>(StringComparer.OrdinalIgnoreCase);

    // NEW: loading indicator for long-running tree rebuilds (bind this to a spinner/overlay in the view)
    private bool _isLoading;
    public bool IsLoading { get => _isLoading; set => Set(ref _isLoading, value); }

    public void Log(string message)
    {
        var line = $"{DateTime.Now:HH:mm:ss.fff} {message}\n";
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

    public MainWindowViewModel()
    {
        NewCommand = new RelayCommand(() => NewSeason());
        OpenCommand = new RelayCommand(async () => await OpenJsonAsync());
        SaveAsCommand = new RelayCommand(async () => await SaveAsJsonAsync());

        Log("MainWindowViewModel ctor reached.");
        NewSeason();
    }

    public void SetSelectedTreeItem(TreeItemVm? item)
    {
        if (item?.Model is null)
        {
            SelectionDetails = item?.Title ?? "";
            return;
        }

        SelectionDetails = JsonSerializer.Serialize(item.Model, new JsonSerializerOptions { WriteIndented = true });
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

        var group = _doc.Groups[0];
        var location = _doc.Locations[0];
        var position = _doc.Positions[0];
        var action = _doc.Actions[0];

        _doc.Poses.Add(new PoseModel { Group = group, Location = location, Index = 0, Position = position, Action = action });
        _doc.Events.Add(new EventModel { Name = "event1", Type = "await", Parallel = false, Pose = "pose1" });
        _doc.Fixtures.Add(new FixtureSchemaModel { Type = "fixture", Index = 0, Translation = new TranslationModel { Position = new() { 0d, 0d, 0d }, PositionUnits = "inches", Rotation = 0d, RotationUnits = "degrees" } });
        _doc.Targets.Add(new TargetSchemaModel { Module = "module1", Translation = new TranslationModel { Position = new() { 0d, 0d, 0d }, PositionUnits = "inches" } });
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
        var json = JsonSerializer.Serialize(_doc, new JsonSerializerOptions { WriteIndented = true });
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

        // ensure pose names are consistent after load
        PoseHelper.EnsurePoseNames(_doc);

        // If deserialization produced different params representation, ensure _paramsMap is at least populated from _doc
        if (_paramsMap.Count == 0) UpdateParamsMap();

        CurrentJsonPath = dlg.FileName;
        await RebuildTreeAsync();

        Log($"OpenJson: loaded '{dlg.FileName}'.");
    }

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

        var json = JsonSerializer.Serialize(_doc, new JsonSerializerOptions { WriteIndented = true });
        await File.WriteAllTextAsync(dlg.FileName, json);
        CurrentJsonPath = dlg.FileName;

        await RebuildTreeAsync(); // update root label

        Log($"SaveAsJson: wrote '{dlg.FileName}'.");
    }

    // ----- Tree helpers -----

    // NEW: build the entire tree into a root TreeItemVm but do not mutate ObservableCollection.
    private TreeItemVm BuildTreeRoot()
    {
        // Note: this is the same logic that previously lived inside RebuildTree()
        // but it only creates/returns the root TreeItemVm and does not touch UI-bound collections.
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
        root.Children.Add(BuildStringListSection("groups", _doc.Groups));
        root.Children.Add(BuildStringListSection("travelGroups", _doc.TravelGroups));
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
        var root = BuildTreeRoot();

        // install on UI-bound collection (must be done on UI thread)
        TreeRootItems.Clear();
        TreeRootItems.Add(root);
        OnPropertyChanged(nameof(TreeRootItems));

        TreeHelper.RefreshAllOptions(TreeRootItems);

        // logging for diagnostics
        LoggingHelper.LogAllNodeBindings(TreeRootItems, Log);
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

            // Build the tree on a background thread (the expensive part)
            var root = await Task.Run(() => BuildTreeRoot());

            // Marshal applying the constructed root to the UI thread
            // Use Application.Current.Dispatcher to ensure UI-thread update
            if (Application.Current?.Dispatcher != null)
            {
                Application.Current.Dispatcher.Invoke(() =>
                {
                    TreeRootItems.Clear();
                    TreeRootItems.Add(root);
                    OnPropertyChanged(nameof(TreeRootItems));
                    TreeHelper.RefreshAllOptions(TreeRootItems);
                    LoggingHelper.LogAllNodeBindings(TreeRootItems, Log);
                });
            }
            else
            {
                // fallback if no dispatcher available
                TreeRootItems.Clear();
                TreeRootItems.Add(root);
                OnPropertyChanged(nameof(TreeRootItems));
                TreeHelper.RefreshAllOptions(TreeRootItems);
                LoggingHelper.LogAllNodeBindings(TreeRootItems, Log);
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
            TreeHelper.FocusNewUnder(TreeRootItems, "params", key);
        });

        foreach (var kvp in _doc.Params.OrderBy(k => k.Key))
        {
            var raw = kvp.Value?.Raw;

            // If param is an array (preserved as List<object>), create an object node with one child per element.
            if (raw is IList<object> list)
            {
                var node = new TreeItemVm(kvp.Key, TreeItemKind.Object)
                {
                    HasRemove = true,
                    Model = kvp
                };
                node.RemoveCommand = new RelayCommand(() => { _doc.Params.Remove(kvp.Key); _ = RebuildTreeAsync(); });

                // Add command to append a new element
                node.HasAdd = true;
                node.AddCommand = new RelayCommand(() =>
                {
                    list.Add(0d);
                    _ = RebuildTreeAsync();
                    TreeHelper.FocusNewUnder(TreeRootItems, "params", kvp.Key);
                });

                for (var i = 0; i < list.Count; i++)
                {
                    var index = i;
                    // create per-element editor using TreeHelper so it gets the same NumberOrParam behavior/hints
                    var child = TreeHelper.CreateNumberOrParamField(
                        $"[{index}]",
                        kvp.Value!,                     // model (ParamValue)
                        () => list[index],              // getter returns element
                        v => { list[index] = v; },      // setter updates element in-place
                        _paramsMap                       // params map for previews/options
                    );

                    child.HasRemove = true;
                    child.RemoveCommand = new RelayCommand(() =>
                    {
                        if (index >= 0 && index < list.Count) list.RemoveAt(index);
                        _ = RebuildTreeAsync();
                    });

                    node.Children.Add(child);
                }

                sec.Children.Add(node);
            }
            else
            {
                // scalar (number/string) param: single field
                var item = TreeHelper.CreateNumberOrParamField(
                    kvp.Key,
                    kvp.Value!,                      // model (ParamValue)
                    () => kvp.Value?.Raw,            // getter: raw value
                    v => { kvp.Value.Raw = v; },     // setter: update raw
                    _paramsMap                        // params map for previews/options
                );
                item.HasRemove = true;
                item.Model = kvp;
                item.RemoveCommand = new RelayCommand(() => { _doc.Params.Remove(kvp.Key); _ = RebuildTreeAsync(); });

                sec.Children.Add(item);
            }
        }
        return sec;
    }

    private TreeItemVm BuildStringListSection(string name, IList<string> list)
    {
        var sec = new TreeItemVm(name, TreeItemKind.Section) { HasAdd = true };
        sec.AddCommand = new RelayCommand(() =>
        {
            list.Add($"{name}{list.Count + 1}");
            _ = RebuildTreeAsync();           // guarantees pose dropdowns are rebuilt with fresh Options
            TreeHelper.FocusNewUnder(TreeRootItems, name, list.Last());
        });

        for (var i = 0; i < list.Count; i++)
        {
            var index = i;
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
            });

            sec.Children.Add(node);
        }

        return sec;
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

            _ = RebuildTreeAsync();
            TreeHelper.FocusNewUnder(TreeRootItems, "poses");
        });

        foreach (var p in _doc.Poses)
        {
            var node = new TreeItemVm(p.Name ?? $"pose[{p.Group},{p.Location},{p.Index}]", TreeItemKind.Object)
            {
                HasRemove = true,
                RemoveCommand = new RelayCommand(() => { _doc.Poses.Remove(p); PoseHelper.EnsurePoseNames(_doc); _ = RebuildTreeAsync(); }),
                Model = p
            };

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

    private TreeItemVm BuildEventsSection()
    {
        var sec = new TreeItemVm("events", TreeItemKind.Section) { HasAdd = true };
        sec.AddCommand = new RelayCommand(() =>
        {
            var poseName = _doc.Poses.FirstOrDefault()?.Name ?? "pose1";
            _doc.Events.Add(new EventModel { Name = $"event{_doc.Events.Count + 1}", Type = "await", Parallel = false, Pose = poseName });
            _ = RebuildTreeAsync();
            TreeHelper.FocusNewUnder(TreeRootItems, "events");
        });

        foreach (var ev in _doc.Events)
        {
            var node = new TreeItemVm(ev.Name, TreeItemKind.Object)
            {
                HasRemove = true,
                RemoveCommand = new RelayCommand(() => { _doc.Events.Remove(ev); _ = RebuildTreeAsync(); }),
                Model = ev
            };
            var type = new TreeItemVm("type", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.Type, Model = ev };
            type.Options.Add("await"); type.Options.Add("time");
            node.Children.Add(type);

            node.Children.Add(new TreeItemVm("parallel", TreeItemKind.Field) { Editor = TreeEditorKind.Bool, Value = ev.Parallel, Model = ev });
            node.Children.Add(new TreeItemVm("pose", TreeItemKind.Field) { Editor = TreeEditorKind.Text, Value = ev.Pose ?? "", Model = ev });
            node.Children.Add(new TreeItemVm("milliseconds", TreeItemKind.Field) { Editor = TreeEditorKind.NumberOrParam, Value = ev.Milliseconds ?? 0, Model = ev });
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
            _doc.Sequences.Add(new SequenceModel { Name = $"seq{_doc.Sequences.Count + 1}", Events = new() { _doc.Events.First().Name } });
            _ = RebuildTreeAsync();
            TreeHelper.FocusNewUnder(TreeRootItems, "sequences");
        });

        foreach (var s in _doc.Sequences)
        {
            var node = new TreeItemVm(s.Name, TreeItemKind.Object)
            {
                HasRemove = true,
                RemoveCommand = new RelayCommand(() => { _doc.Sequences.Remove(s); _ = RebuildTreeAsync(); }),
                Model = s
            };
            sec.Children.Add(node);
        }
        return sec;
    }

    private TreeItemVm BuildTranslationNode(TranslationModel tr)
    {
        var node = new TreeItemVm("translation", TreeItemKind.Object) { Model = tr };

        // position[0..2] numbers or param strings
        tr.Position ??= new() { 0d, 0d, 0d };
        while (tr.Position.Count < 3) tr.Position.Add(0d);

        // pass params map so the created node can expose a calculated hint while preserving the expression
        node.Children.Add(TreeHelper.CreateNumberOrParamField("position[0]", tr, () => tr.Position[0], v => tr.Position[0] = v, _paramsMap));
        node.Children.Add(TreeHelper.CreateNumberOrParamField("position[1]", tr, () => tr.Position[1], v => tr.Position[1] = v, _paramsMap));
        node.Children.Add(TreeHelper.CreateNumberOrParamField("position[2]", tr, () => tr.Position[2], v => tr.Position[2] = v, _paramsMap));

        var pu = new TreeItemVm("positionUnits", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = tr.PositionUnits ?? "inches", Model = tr };
        pu.Options.Add("meters"); pu.Options.Add("inches");
        node.Children.Add(pu);

        node.Children.Add(TreeHelper.CreateNumberOrParamField("rotation", tr, () => tr.Rotation ?? 0d, v => tr.Rotation = v, _paramsMap));

        var ru = new TreeItemVm("rotationUnits", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = tr.RotationUnits ?? "degrees", Model = tr };
        ru.Options.Add("radians"); ru.Options.Add("degrees");
        node.Children.Add(ru);

        return node;
    }

    private void AddFixtureVm(TreeItemVm fixturesSection, FixtureSchemaModel f)
    {
        var node = new TreeItemVm($"{f.Type}:{f.Index}", TreeItemKind.Object)
        {
            IsCaptionEditable = true,
            Model = f,
            HasRemove = true
        };
        node.RemoveCommand = new RelayCommand(() =>
        {
            _doc.Fixtures.Remove(f);
            fixturesSection.Children.Remove(node);
        });

        // Editable fields per fixture.schema.json
        node.Children.Add(TreeHelper.CreateTextField("type", f, () => f.Type, v => f.Type = v));
        node.Children.Add(TreeHelper.CreateNumberOrParamField("index", f, () => f.Index, v => f.Index = (int)v, _paramsMap));

        // translation (oneOf translation|derivedFrom); create by default
        f.Translation ??= new TranslationModel { Position = new() { 0d, 0d, 0d }, PositionUnits = "inches", RotationUnits = "degrees" };
        node.Children.Add(TreeHelper.CreateTranslationNode(f.Translation, ParamKeys));

        fixturesSection.Children.Add(node);

        fixturesSection.IsExpanded = true;
        node.IsSelected = true;
    }

    private void AddTargetVm(TreeItemVm targetsSection, TargetSchemaModel t)
    {
        var node = new TreeItemVm(t.Module, TreeItemKind.Object)
        {
            IsCaptionEditable = true,
            Model = t,
            HasRemove = true
        };
        node.RemoveCommand = new RelayCommand(() =>
        {
            _doc.Targets.Remove(t);
            targetsSection.Children.Remove(node);
        });

        // Editable fields per target.schema.json
        node.Children.Add(TreeHelper.CreateTextField("module", t, () => t.Module, v => t.Module = v));
        node.Children.Add(TreeHelper.CreateTextField("group", t, () => t.Group ?? "", v => t.Group = string.IsNullOrWhiteSpace(v) ? null : v));
        node.Children.Add(TreeHelper.CreateTextField("location", t, () => t.Location ?? "", v => t.Location = string.IsNullOrWhiteSpace(v) ? null : v));
        node.Children.Add(TreeHelper.CreateNumberOrParamField("index", t, () => t.Index ?? -1, v => t.Index = Convert.ToInt32(v), _paramsMap));
        node.Children.Add(TreeHelper.CreateTextField("position", t, () => t.Position ?? "", v => t.Position = string.IsNullOrWhiteSpace(v) ? null : v));
        node.Children.Add(TreeHelper.CreateTextField("action", t, () => t.Action ?? "", v => t.Action = string.IsNullOrWhiteSpace(v) ? null : v));

        // oneOf: measurement | state | translation | fixture (seed translation by default)
        t.Translation ??= new TranslationModel { Position = new() { 0d, 0d, 0d }, PositionUnits = "inches", RotationUnits = "degrees" };
        node.Children.Add(TreeHelper.CreateTranslationNode(t.Translation, ParamKeys));

        targetsSection.Children.Add(node);

        targetsSection.IsExpanded = true;
        node.IsSelected = true;
    }

    // wrappers removed; VM now calls TreeHelper.Create* / TreeHelper.FocusNewUnder directly.

}
