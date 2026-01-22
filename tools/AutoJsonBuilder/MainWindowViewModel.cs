using System.Collections.ObjectModel;
using System.IO;
using System.Text.Json;
using System.Windows.Input;
using Microsoft.Win32;
using AutoJsonBuilder.Models;
using System.Linq;
using AutoJsonBuilder.Helpers; // <-- added

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
    private IDictionary<string, double> _paramsMap = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);

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
        foreach (var kv in _doc.Params)
        {
            // tolerate numeric types stored in the model
            try { _paramsMap[kv.Key] = Convert.ToDouble(kv.Value); }
            catch { _paramsMap[kv.Key] = 0d; }
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
        RebuildTree();

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
        RebuildTree();

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

        // Parse once to extract params map for resolving param references
        Dictionary<string, double> paramsMap;
        using (var doc = JsonDocument.Parse(json))
        {
            paramsMap = ParamAwareConverters.BuildParamsMap(doc.RootElement);
        }

        // use flexible options so numeric fields can accept numbers, numeric-strings, arrays or param refs
        var opts = new JsonSerializerOptions { PropertyNameCaseInsensitive = true };
        // keep flexible dictionary converter for the params object itself
        opts.Converters.Add(new FlexibleDoubleDictionaryConverter());

        _doc = JsonSerializer.Deserialize<AutoDefinitionModel>(json, opts)
               ?? throw new InvalidOperationException("Invalid JSON file.");

        // moved
        PoseHelper.EnsurePoseBackingLists(_doc);

        // ensure pose names are consistent after load
        PoseHelper.EnsurePoseNames(_doc);

        // update params map for UI calculations
        UpdateParamsMap();

        CurrentJsonPath = dlg.FileName;
        RebuildTree();

        Log($"OpenJson: loaded '{dlg.FileName}'.");
    }

    // Return the first 'deploy' folder found under the solution, or a sensible fallback.
    public string? GetDefaultDeployFolder()
    {
        // delegate to FileHelper
        return FileHelper.GetDefaultDeployFolder();
    }

    private async Task SaveAsJsonAsync()
    {
        var dlg = new SaveFileDialog
        {
            Title = "Save auto JSON",
            Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*",
            FileName = Path.GetFileName(CurrentJsonPath) ?? "auto.json",
            OverwritePrompt = true,
            InitialDirectory = GetDefaultDeployFolder() ?? Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments)
        };
        if (dlg.ShowDialog() != true) return;

        var json = JsonSerializer.Serialize(_doc, new JsonSerializerOptions { WriteIndented = true });
        await File.WriteAllTextAsync(dlg.FileName, json);
        CurrentJsonPath = dlg.FileName;

        RebuildTree(); // update root label

        Log($"SaveAsJson: wrote '{dlg.FileName}'.");
    }

    // ----- Tree helpers -----

    private void RebuildTree()
    {
        Log("RebuildTree()");
        TreeRootItems.Clear();
        ParamKeys.Clear();
        foreach (var k in _doc.Params.Keys.OrderBy(x => x))
            ParamKeys.Add(k);

        // ensure params map is current before creating nodes
        UpdateParamsMap();

        var rootTitle = Path.GetFileName(CurrentJsonPath) ?? "(unsaved)";
        var root = new TreeItemVm(rootTitle, TreeItemKind.Root) { IsExpanded = true };

        // required scalars
        root.Children.Add(TreeHelper.CreateField("season", _doc, () => _doc.Season, v => _doc.Season = v));
        root.Children.Add(TreeHelper.CreateField("version", _doc, () => _doc.Version, v => _doc.Version = v));
        root.Children.Add(TreeHelper.CreateField("description", _doc, () => _doc.Description, v => _doc.Description = v));

        // sections
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

        TreeRootItems.Add(root);
        OnPropertyChanged(nameof(TreeRootItems));

        RefreshAllOptions();

        // NEW: log resolved node values so we can see what the UI should be binding to
        // delegated to LoggingHelper (pass our Log method)
        LoggingHelper.LogAllNodeBindings(TreeRootItems, Log);
    }

    private void RefreshAllOptions()
    {
        // delegated to TreeHelper
        TreeHelper.RefreshAllOptions(TreeRootItems);
    }

    private TreeItemVm BuildParamsSection()
    {
        var sec = new TreeItemVm("params", TreeItemKind.Section) { HasAdd = true };
        sec.AddCommand = new RelayCommand(() =>
        {
            var key = $"param{_doc.Params.Count + 1}";
            _doc.Params[key] = 0;
            RebuildTree();
            TreeHelper.FocusNewUnder(TreeRootItems, "params", key);
        });

        foreach (var kvp in _doc.Params.OrderBy(k => k.Key))
        {
            var item = new TreeItemVm(kvp.Key, TreeItemKind.Field)
            {
                Editor = TreeEditorKind.NumberOrParam,
                Value = kvp.Value,
                HasRemove = true,
                Model = kvp
            };
            item.RemoveCommand = new RelayCommand(() => { _doc.Params.Remove(kvp.Key); RebuildTree(); });
            sec.Children.Add(item);
        }
        return sec;
    }

    private TreeItemVm BuildStringListSection(string name, IList<string> list)
    {
        var sec = new TreeItemVm(name, TreeItemKind.Section) { HasAdd = true };
        sec.AddCommand = new RelayCommand(() =>
        {
            list.Add($"{name}{list.Count + 1}");
            RebuildTree();           // guarantees pose dropdowns are rebuilt with fresh Options
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
                RebuildTree();       // guarantees pose dropdowns are rebuilt with fresh Options
            };

            node.RemoveCommand = new RelayCommand(() =>
            {
                list.RemoveAt(index);
                RebuildTree();       // guarantees pose dropdowns are rebuilt with fresh Options
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

            RebuildTree();
            TreeHelper.FocusNewUnder(TreeRootItems, "poses");
        });

        foreach (var p in _doc.Poses)
        {
            var node = new TreeItemVm(p.Name ?? $"pose[{p.Group},{p.Location},{p.Index}]", TreeItemKind.Object)
            {
                HasRemove = true,
                RemoveCommand = new RelayCommand(() => { _doc.Poses.Remove(p); PoseHelper.EnsurePoseNames(_doc); RebuildTree(); }),
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
            RebuildTree();
            TreeHelper.FocusNewUnder(TreeRootItems, "events");
        });

        foreach (var ev in _doc.Events)
        {
            var node = new TreeItemVm(ev.Name, TreeItemKind.Object)
            {
                HasRemove = true,
                RemoveCommand = new RelayCommand(() => { _doc.Events.Remove(ev); RebuildTree(); }),
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
            RebuildTree();
            TreeHelper.FocusNewUnder(TreeRootItems, "sequences");
        });

        foreach (var s in _doc.Sequences)
        {
            var node = new TreeItemVm(s.Name, TreeItemKind.Object)
            {
                HasRemove = true,
                RemoveCommand = new RelayCommand(() => { _doc.Sequences.Remove(s); RebuildTree(); }),
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
