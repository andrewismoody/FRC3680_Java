using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using AutoJsonBuilder.Models;

namespace AutoJsonBuilder.Helpers;

// Helper that builds the TreeItemVm tree from the document.
// Accepts delegates for VM-level actions (rebuild, rename, focus, trigger-save, commit pose caption).
public static class TreeBuildHelper
{
	// Delegate types
	public delegate Task AsyncRebuildDelegate();
	public delegate void TriggerSaveDelegate();
	public delegate void FocusDelegate(string sectionTitle, string? childTitle);
	public delegate Task RenameDelegate(string oldName, string newName);
	public delegate Task CommitPoseCaptionDelegate(TreeItemVm? node);

	public static TreeItemVm BuildTreeRoot(
		AutoDefinitionModel doc,
		IEnumerable<string> paramKeys,
		IDictionary<string, object> paramsMap,
		AsyncRebuildDelegate rebuildAsync,
		TriggerSaveDelegate triggerModelChangedImmediate,
		FocusDelegate focusNewUnder,
		RenameDelegate renameParamAsync,
		RenameDelegate renamePoseAsync,
		RenameDelegate renameEventAsync,
		RenameDelegate renameSequenceAsync,
		CommitPoseCaptionDelegate commitPoseCaptionAsync)
	{
		// Keep same high-level structure as original BuildTreeRoot
		var rootTitle = System.IO.Path.GetFileName((doc as object) == null ? null : null) ?? "(unsaved)"; // placeholder; VM will set proper root later/override
		// Prefer to let VM supply file name via CurrentJsonPath; the VM wrapper already called EvaluateParamsOnly before calling here.
		var root = new TreeItemVm("(root)", TreeItemKind.Root) { IsExpanded = true };

		// required scalars (use TreeHelper factory methods)
		root.Children.Add(TreeHelper.CreateField("season", doc, () => doc.Season ?? "", v => doc.Season = v));
		root.Children.Add(TreeHelper.CreateField("version", doc, () => doc.Version ?? "", v => doc.Version = v));
		root.Children.Add(TreeHelper.CreateField("description", doc, () => doc.Description ?? "", v => doc.Description = v));

		// sections
		root.Children.Add(BuildParamsSection(doc, paramsMap, paramKeys, rebuildAsync, triggerModelChangedImmediate, focusNewUnder, renameParamAsync));
		root.Children.Add(BuildStringListSection("modules", doc.Modules, rebuildAsync, triggerModelChangedImmediate, focusNewUnder));
		root.Children.Add(BuildStringListSection("groups", doc.Groups, rebuildAsync, triggerModelChangedImmediate, focusNewUnder));
		root.Children.Add(BuildStringListSection("travelGroups", doc.TravelGroups, rebuildAsync, triggerModelChangedImmediate, focusNewUnder, doc.Groups));
		root.Children.Add(BuildStringListSection("locations", doc.Locations, rebuildAsync, triggerModelChangedImmediate, focusNewUnder));
		root.Children.Add(BuildStringListSection("positions", doc.Positions, rebuildAsync, triggerModelChangedImmediate, focusNewUnder));
		root.Children.Add(BuildStringListSection("actions", doc.Actions, rebuildAsync, triggerModelChangedImmediate, focusNewUnder));
		root.Children.Add(BuildPosesSection(doc, rebuildAsync, triggerModelChangedImmediate, focusNewUnder, renamePoseAsync, commitPoseCaptionAsync));
		root.Children.Add(BuildEventsSection(doc, rebuildAsync, triggerModelChangedImmediate, focusNewUnder, renameEventAsync));
		root.Children.Add(BuildFixturesSection(doc, rebuildAsync, triggerModelChangedImmediate, focusNewUnder));
		root.Children.Add(BuildTargetsSection(doc, paramKeys, rebuildAsync, triggerModelChangedImmediate, focusNewUnder));
		root.Children.Add(BuildSequencesSection(doc, rebuildAsync, triggerModelChangedImmediate, focusNewUnder, renameSequenceAsync));

		return root;
	}

	// ---- section builders (extracted from VM and adapted to use delegates) ----

	private static TreeItemVm BuildParamsSection(
		AutoDefinitionModel doc,
		IDictionary<string, object> paramsMap,
		IEnumerable<string> paramKeys,
		AsyncRebuildDelegate rebuildAsync,
		TriggerSaveDelegate triggerModelChangedImmediate,
		FocusDelegate focusNewUnder,
		RenameDelegate renameParamAsync)
	{
		var sec = new TreeItemVm("params", TreeItemKind.Section) { HasAdd = true };
		sec.AddCommand = new RelayCommand(() =>
		{
			var key = $"param{doc.Params.Count + 1}";
			doc.Params[key] = new AutoJsonBuilder.Models.ParamValue(0d);
			_ = rebuildAsync();
			triggerModelChangedImmediate();
			focusNewUnder("params", key);
		});

		foreach (var kvp in doc.Params.OrderBy(k => k.Key))
		{
			var raw = kvp.Value?.Raw;
			if (raw is IList<object> list)
			{
				var originalKey = kvp.Key;
				var originalVal = kvp.Value;
				var node = new TreeItemVm(kvp.Key, TreeItemKind.Object) { HasRemove = true, Model = kvp, IsCaptionEditable = true };
				node.PropertyChanged += async (_, e) =>
				{
					if (e.PropertyName != nameof(TreeItemVm.Title)) return;
					var newKey = node.Title?.Trim() ?? "";
					if (string.IsNullOrWhiteSpace(newKey) || newKey == originalKey) { node.Title = originalKey; return; }
					if (doc.Params.ContainsKey(newKey)) { node.Title = originalKey; return; }
					_ = renameParamAsync(originalKey, newKey);
				};
				node.RemoveCommand = new RelayCommand(() => { doc.Params.Remove(originalKey); _ = rebuildAsync(); triggerModelChangedImmediate(); });

				node.HasAdd = list.Count < 3;
				node.AddCommand = new RelayCommand(() =>
				{
					if (list.Count >= 3) return;
					list.Add(0d);
					_ = rebuildAsync();
					triggerModelChangedImmediate();
					focusNewUnder("params", originalKey);
				});

				for (var i = 0; i < list.Count; i++)
				{
					var index = i;
					var child = TreeHelper.CreateNumberOrParamField(
						TreeHelper.GetArrayIndexLabel(index),
						kvp.Value!,
						() => list[index],
						v => list[index] = v,
						paramsMap,
						captionEditable: true
					);
					child.Editor = TreeEditorKind.Text;
					child.Options.Clear();

					if (list.Count >= 2 && index == list.Count - 1)
					{
						child.HasRemove = true;
						var idx = index;
						child.RemoveCommand = new RelayCommand(() =>
						{
							if (list.Count == 2)
							{
								var first = list.Count > 0 ? list[0] : 0d;
								originalVal.Raw = first!;
								doc.Params[originalKey] = originalVal!;
							}
							else
							{
								if (idx >= 0 && idx < list.Count) list.RemoveAt(idx);
							}
							// update external map if VM expects it
							_ = rebuildAsync();
							triggerModelChangedImmediate();
							focusNewUnder("params", originalKey);
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
				var node = new TreeItemVm(originalKey, TreeItemKind.Object) { HasRemove = true, Model = kvp };
				node.RemoveCommand = new RelayCommand(() => { doc.Params.Remove(originalKey); _ = rebuildAsync(); triggerModelChangedImmediate(); });

				node.IsCaptionEditable = true;
				node.PropertyChanged += async (_, e) =>
				{
					if (e.PropertyName != nameof(TreeItemVm.Title)) return;
					var newKey = node.Title?.Trim() ?? "";
					if (string.IsNullOrWhiteSpace(newKey) || newKey == originalKey) { node.Title = originalKey; return; }
					if (doc.Params.ContainsKey(newKey)) { node.Title = originalKey; return; }
					_ = renameParamAsync(originalKey, newKey);
				};

				node.HasAdd = true;
				node.AddCommand = new RelayCommand(() =>
				{
					var rawVal = originalVal?.Raw;
					if (rawVal is IList<object>) return;
					var newList = new List<object>();
					newList.Add(rawVal ?? 0d);
					newList.Add(0d);
					originalVal.Raw = newList;
					doc.Params[originalKey] = originalVal!;
					_ = rebuildAsync();
					triggerModelChangedImmediate();
					focusNewUnder("params", originalKey);
				});

				var child = TreeHelper.CreateNumberOrParamField(
					"value",
					kvp.Value!,
					() => kvp.Value?.Raw!,
					v => kvp.Value.Raw = v,
					paramsMap,
					captionEditable: false
				);
				child.Editor = TreeEditorKind.Text;
				child.Options.Clear();
				child.Model = kvp;

				node.Children.Add(child);
				sec.Children.Add(node);
			}
		}

		// update paramKeys observable on VM will be handled by VM after rebuild; helper returns the tree only.
		return sec;
	}

	private static TreeItemVm BuildStringListSection(string name, IList<string> list, AsyncRebuildDelegate rebuildAsync, TriggerSaveDelegate triggerModelChangedImmediate, FocusDelegate focusNewUnder, IList<string>? optionsSource = null)
	{
		var sec = new TreeItemVm(name, TreeItemKind.Section) { HasAdd = true };
		sec.AddCommand = new RelayCommand(() =>
		{
			var baseName = GetSingularName(name);
			var newVal = optionsSource != null && optionsSource.Count > 0 ? optionsSource[0] : $"{baseName}{list.Count + 1}";
			list.Add(newVal);
			_ = rebuildAsync();
			triggerModelChangedImmediate();
			focusNewUnder(name, list.Last());
		});

		for (var i = 0; i < list.Count; i++)
		{
			var index = i;
			if (optionsSource != null)
			{
				var field = TreeHelper.CreateListField("", list, () => list[index], v => list[index] = v, optionsSource);
				field.IsCaptionEditable = false;
				field.HasRemove = true;
				field.Model = list;
				field.RemoveCommand = new RelayCommand(() =>
				{
					if (index >= 0 && index < list.Count) list.RemoveAt(index);
					_ = rebuildAsync();
					triggerModelChangedImmediate();
				});
				sec.Children.Add(field);
			}
			else
			{
				var node = new TreeItemVm(list[index], TreeItemKind.Object) { IsCaptionEditable = true, Editor = TreeEditorKind.None, HasRemove = true };
				node.PropertyChanged += (_, e) =>
				{
					if (e.PropertyName != nameof(TreeItemVm.Title)) return;
					var newValue = node.Title?.Trim() ?? "";
					list[index] = newValue;
					_ = rebuildAsync();
				};
				node.RemoveCommand = new RelayCommand(() =>
				{
					list.RemoveAt(index);
					_ = rebuildAsync();
					triggerModelChangedImmediate();
				});
				sec.Children.Add(node);
			}
		}
		return sec;
	}

	private static TreeItemVm BuildPosesSection(AutoDefinitionModel doc, AsyncRebuildDelegate rebuildAsync, TriggerSaveDelegate triggerModelChangedImmediate, FocusDelegate focusNewUnder, RenameDelegate renamePoseAsync, CommitPoseCaptionDelegate commitPoseCaptionAsync)
	{
		var sec = new TreeItemVm("poses", TreeItemKind.Section) { HasAdd = true };
		sec.AddCommand = new RelayCommand(() =>
		{
			var newPose = new PoseModel
			{
				Group = doc.Groups.FirstOrDefault() ?? "",
				Location = doc.Locations.FirstOrDefault() ?? "",
				Index = 0,
				Position = doc.Positions.FirstOrDefault() ?? "",
				Action = doc.Actions.FirstOrDefault() ?? ""
			};
			doc.Poses.Add(newPose);
			PoseHelper.EnsurePoseNames(doc);
			_ = rebuildAsync();
			triggerModelChangedImmediate();
			focusNewUnder("poses", null);
		});

		foreach (var p in doc.Poses)
		{
			var originalTitle = p.Name ?? $"pose[{p.Group},{p.Location},{p.Index}]";
			var node = new TreeItemVm(originalTitle, TreeItemKind.Object)
			{
				HasRemove = true,
				RemoveCommand = new RelayCommand(() => { doc.Poses.Remove(p); PoseHelper.EnsurePoseNames(doc); _ = rebuildAsync(); triggerModelChangedImmediate(); }),
				Model = p,
				IsCaptionEditable = true
			};

			// Do not wire Title changes here; VM will call commit pose caption on LostFocus (delegate provided).
			node.Children.Add(TreeHelper.CreatePoseDropdown("group", p, () => p.Group, v => p.Group = v, doc.Groups));
			node.Children.Add(TreeHelper.CreatePoseDropdown("location", p, () => p.Location, v => p.Location = v, doc.Locations));
			node.Children.Add(TreeHelper.CreateIntField("index", p, () => p.Index, v => p.Index = v));
			node.Children.Add(TreeHelper.CreatePoseDropdown("position", p, () => p.Position, v => p.Position = v, doc.Positions));
			node.Children.Add(TreeHelper.CreatePoseDropdown("action", p, () => p.Action, v => p.Action = v, doc.Actions));

			sec.Children.Add(node);
		}
		return sec;
	}

	private static TreeItemVm BuildEventsSection(AutoDefinitionModel doc, AsyncRebuildDelegate rebuildAsync, TriggerSaveDelegate triggerModelChangedImmediate, FocusDelegate focusNewUnder, RenameDelegate renameEventAsync)
	{
		var sec = new TreeItemVm("events", TreeItemKind.Section) { HasAdd = true };
		sec.AddCommand = new RelayCommand(() =>
		{
			var poseName = doc.Poses.FirstOrDefault()?.Name ?? "pose1";
			var ev = new EventModel
			{
				Name = $"event{doc.Events.Count + 1}",
				Type = "await",
				Parallel = false,
				Pose = poseName,
				TriggerType = "none",
				TriggerValue = false,
				TriggerModule = null,
				TriggerInvert = false
			};
			doc.Events.Add(ev);
			_ = rebuildAsync();
			triggerModelChangedImmediate();
			focusNewUnder("events", null);
		});

		foreach (var ev in doc.Events)
		{
			var originalTitle = ev.Name;
			var node = new TreeItemVm(ev.Name, TreeItemKind.Object) { HasRemove = true, RemoveCommand = new RelayCommand(() => { doc.Events.Remove(ev); _ = rebuildAsync(); }), Model = ev, IsCaptionEditable = true };
			node.PropertyChanged += async (_, e) =>
			{
				if (e.PropertyName != nameof(TreeItemVm.Title)) return;
				var newTitle = node.Title?.Trim() ?? "";
				var oldName = originalTitle ?? "";
				if (string.IsNullOrWhiteSpace(newTitle)) { node.Title = oldName; return; }
				if (string.Equals(newTitle, oldName, StringComparison.OrdinalIgnoreCase)) return;
				if (doc.Events.Any(x => string.Equals(x.Name, newTitle, StringComparison.OrdinalIgnoreCase))) { node.Title = oldName; return; }
				_ = renameEventAsync(oldName, newTitle);
			};

			var type = new TreeItemVm("type", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.Type, Model = ev };
			type.Options.Add("await"); type.Options.Add("time");
			type.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) { ev.Type = type.Value?.ToString() ?? "await"; triggerModelChangedImmediate(); } };
			node.Children.Add(type);

			var parallel = new TreeItemVm("parallel", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.Parallel ? "true" : "false", Model = ev };
			parallel.Options.Add("true"); parallel.Options.Add("false");
			parallel.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) { var s = parallel.Value?.ToString(); ev.Parallel = string.Equals(s, "true", StringComparison.OrdinalIgnoreCase); triggerModelChangedImmediate(); } };
			node.Children.Add(parallel);

			var poseField = new TreeItemVm("pose", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.Pose ?? "", Model = ev };
			poseField.Options.Clear(); poseField.Options.Add("");
			foreach (var pName in doc.Poses.Select(p => p.Name ?? "")) poseField.Options.Add(pName);
			poseField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) { var sel = poseField.Value?.ToString() ?? ""; ev.Pose = string.IsNullOrWhiteSpace(sel) ? null : sel; triggerModelChangedImmediate(); } };
			node.Children.Add(poseField);

			var ms = new TreeItemVm("milliseconds", TreeItemKind.Field) { Editor = TreeEditorKind.NumberOrParam, Value = ev.Milliseconds ?? 0, Model = ev };
			ms.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) { ev.Milliseconds = Convert.ToInt32(ms.Value ?? 0); triggerModelChangedImmediate(); } };
			node.Children.Add(ms);

			var tt = new TreeItemVm("triggerType", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.TriggerType ?? "none", Model = ev };
			tt.Options.Add("boolean"); tt.Options.Add("none");
			tt.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) { ev.TriggerType = tt.Value?.ToString(); triggerModelChangedImmediate(); } };
			node.Children.Add(tt);

			var tv = new TreeItemVm("triggerValue", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.TriggerValue, Model = ev };
			tv.Options.Add("true"); tv.Options.Add("false");
			tv.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) { var s = tv.Value?.ToString(); ev.TriggerValue = string.Equals(s, "true", StringComparison.OrdinalIgnoreCase); triggerModelChangedImmediate(); } };
			node.Children.Add(tv);

			var tm = new TreeItemVm("triggerModule", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.TriggerModule ?? "", Model = ev };
			tm.Options.Clear(); tm.Options.Add("");
			foreach (var m in doc.Modules) tm.Options.Add(m ?? "");
			tm.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) { var sel = tm.Value?.ToString() ?? ""; ev.TriggerModule = string.IsNullOrWhiteSpace(sel) ? null : sel; triggerModelChangedImmediate(); } };
			node.Children.Add(tm);

			var ti = new TreeItemVm("triggerInvert", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = ev.TriggerInvert ? "true" : "false", Model = ev };
			ti.Options.Add("true"); ti.Options.Add("false");
			ti.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) { var s = ti.Value?.ToString(); ev.TriggerInvert = string.Equals(s, "true", StringComparison.OrdinalIgnoreCase); triggerModelChangedImmediate(); } };
			node.Children.Add(ti);

			sec.Children.Add(node);
		}
		return sec;
	}

	private static TreeItemVm BuildFixturesSection(AutoDefinitionModel doc, AsyncRebuildDelegate rebuildAsync, TriggerSaveDelegate triggerModelChangedImmediate, FocusDelegate focusNewUnder)
	{
		var sec = new TreeItemVm("fixtures", TreeItemKind.Section) { HasAdd = true };
		sec.AddCommand = new RelayCommand(() =>
		{
			var f = new FixtureSchemaModel { Type = "fixture", Index = doc.Fixtures.Count };
			doc.Fixtures.Add(f);
			AddFixtureVm(sec, doc, f, rebuildAsync, triggerModelChangedImmediate, focusNewUnder);
			triggerModelChangedImmediate();
		});
		foreach (var f in doc.Fixtures) AddFixtureVm(sec, doc, f, rebuildAsync, triggerModelChangedImmediate, focusNewUnder);
		return sec;
	}

	private static void AddFixtureVm(TreeItemVm fixturesSection, AutoDefinitionModel doc, FixtureSchemaModel f, AsyncRebuildDelegate rebuildAsync, TriggerSaveDelegate triggerModelChangedImmediate, FocusDelegate focusNewUnder)
	{
		var node = new TreeItemVm($"{f.Type}:{f.Index}", TreeItemKind.Object) { IsCaptionEditable = false, Model = f, HasRemove = true };
		node.RemoveCommand = new RelayCommand(() => { doc.Fixtures.Remove(f); fixturesSection.Children.Remove(node); triggerModelChangedImmediate(); });
		node.Children.Add(TreeHelper.CreateTextField("type", f, () => f.Type, v => f.Type = v));
		node.Children.Add(TreeHelper.CreateIntField("index", f, () => f.Index, v => f.Index = v));

		var fixtureOptionsProvider = new Func<IEnumerable<string>>(() => doc.Fixtures.Where(x => !object.ReferenceEquals(x, f)).Select(x => $"{x.Type}:{x.Index}").Distinct());

		var source = new TreeItemVm("source", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Model = f };
		source.Options.Add("translation"); source.Options.Add("derivedFrom");
		source.Value = f.DerivedFrom != null ? "derivedFrom" : "translation";
		node.Children.Add(source);

		void AddActiveChild()
		{
			var existing = node.Children.FirstOrDefault(c => c.Title == "translation" || c.Title == "derivedFrom");
			if (existing != null) node.Children.Remove(existing);
			if (string.Equals(source.Value?.ToString(), "derivedFrom", StringComparison.OrdinalIgnoreCase))
			{
				if (f.DerivedFrom == null) f.DerivedFrom = new DerivedFromModel();
				node.Children.Add(TreeHelper.CreateDerivedFromNode(f.DerivedFrom!, paramKeys: Enumerable.Empty<string>(), fixtureOptionsProvider));
				f.Translation = null;
			}
			else
			{
				f.Translation ??= new TranslationModel { Position = new() { 0d, 0d, 0d }, PositionUnits = "inches", RotationUnits = "degrees" };
				node.Children.Add(TreeHelper.CreateTranslationNode(f.Translation!, Enumerable.Empty<string>()));
				f.DerivedFrom = null;
			}
		}

		AddActiveChild();

		source.PropertyChanged += async (_, e) =>
		{
			if (e.PropertyName != nameof(TreeItemVm.Value)) return;
			AddActiveChild();
			triggerModelChangedImmediate();
			_ = rebuildAsync();
		};

		fixturesSection.Children.Add(node);
		fixturesSection.IsExpanded = true;
		node.IsSelected = true;
	}

	private static TreeItemVm BuildTargetsSection(AutoDefinitionModel doc, IEnumerable<string> paramKeys, AsyncRebuildDelegate rebuildAsync, TriggerSaveDelegate triggerModelChangedImmediate, FocusDelegate focusNewUnder)
	{
		var sec = new TreeItemVm("targets", TreeItemKind.Section) { HasAdd = true };
		sec.AddCommand = new RelayCommand(() =>
		{
			var t = new TargetSchemaModel { Module = $"module{doc.Targets.Count + 1}" };
			doc.Targets.Add(t);
			// add node
			AddTargetVm(sec, doc, t, paramKeys, rebuildAsync, triggerModelChangedImmediate, focusNewUnder);
			triggerModelChangedImmediate();
		});
		foreach (var t in doc.Targets) AddTargetVm(sec, doc, t, paramKeys, rebuildAsync, triggerModelChangedImmediate, focusNewUnder);
		return sec;
	}

	private static void AddTargetVm(TreeItemVm targetsSection, AutoDefinitionModel doc, TargetSchemaModel t, IEnumerable<string> paramKeys, AsyncRebuildDelegate rebuildAsync, TriggerSaveDelegate triggerModelChangedImmediate, FocusDelegate focusNewUnder)
	{
		var node = new TreeItemVm("", TreeItemKind.Object) { IsCaptionEditable = false, Model = t, HasRemove = true };
		node.RemoveCommand = new RelayCommand(() => { doc.Targets.Remove(t); targetsSection.Children.Remove(node); triggerModelChangedImmediate(); });

		var moduleField = TreeHelper.CreateListField("module", t, () => t.Module ?? "", v => t.Module = string.IsNullOrWhiteSpace(v) ? null : v, doc.Modules);
		var groupField = TreeHelper.CreatePoseDropdown("group", t, () => t.Group ?? "", v => t.Group = v, doc.Groups);
		var locationField = TreeHelper.CreatePoseDropdown("location", t, () => t.Location ?? "", v => t.Location = v, doc.Locations);
		var indexField = TreeHelper.CreateIntField("index", t, () => t.Index ?? -1, v => t.Index = v);
		var positionField = TreeHelper.CreatePoseDropdown("position", t, () => t.Position ?? "", v => t.Position = v, doc.Positions);
		var actionField = TreeHelper.CreatePoseDropdown("action", t, () => t.Action ?? "", v => t.Action = v, doc.Actions);

		node.Children.Add(moduleField); node.Children.Add(groupField); node.Children.Add(locationField);
		node.Children.Add(indexField); node.Children.Add(positionField); node.Children.Add(actionField);

		var fixtureOptionsProvider = new Func<IEnumerable<string>>(() => doc.Fixtures.Select(x => $"{x.Type}:{x.Index}").Distinct());

		var kindField = new TreeItemVm("kind", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Model = t };
		kindField.Options.Add("translation"); kindField.Options.Add("fixture");
		kindField.Value = t.Fixture != null ? "fixture" : "translation";
		node.Children.Add(kindField);

		void AddActiveChild()
		{
			var existing = node.Children.FirstOrDefault(c => c.Title == "translation" || c.Title == "fixture");
			if (existing != null) node.Children.Remove(existing);
			if (string.Equals(kindField.Value?.ToString(), "fixture", StringComparison.OrdinalIgnoreCase))
			{
				t.Fixture ??= new FixtureRefModel();
				node.Children.Add(TreeHelper.CreateFixtureRefNode("fixture", t.Fixture,
					() => t.Fixture?.Type ?? "",
					v => t.Fixture!.Type = v,
					() => t.Fixture?.Index ?? 0,
					v => t.Fixture!.Index = v,
					fixtureOptionsProvider));
				t.Translation = null;
			}
			else
			{
				t.Translation ??= new TranslationModel { Position = new() { 0d, 0d, 0d }, PositionUnits = "inches", RotationUnits = "degrees" };
				node.Children.Add(TreeHelper.CreateTranslationNode(t.Translation, paramKeys));
				t.Fixture = null;
			}
		}
		AddActiveChild();

		void UpdateTitle()
		{
			string body;
			if (t.Fixture != null)
			{
				var ft = t.Fixture.Type ?? "(type)";
				var fi = t.Fixture.Index;
				body = $"fixture {ft}:{fi}";
			}
			else
			{
				var parts = new List<string>();
				if (!string.IsNullOrWhiteSpace(t.Group)) parts.Add(t.Group);
				if (!string.IsNullOrWhiteSpace(t.Location))
				{
					if (parts.Count > 0) parts[parts.Count - 1] = parts.Last() + "/" + t.Location;
					else parts.Add(t.Location);
				}
				if (t.Index.HasValue)
				{
					var idxText = t.Index.Value.ToString();
					if (parts.Count > 0) parts[parts.Count - 1] = parts.Last() + $"[{idxText}]";
					else parts.Add($"[{idxText}]");
				}
				if (!string.IsNullOrWhiteSpace(t.Position)) parts.Add(t.Position);
				if (!string.IsNullOrWhiteSpace(t.Action)) parts.Add(t.Action);
				body = string.Join(" ", parts);
			}
			var modulePart = string.IsNullOrWhiteSpace(t.Module) ? null : t.Module;
			if (string.IsNullOrWhiteSpace(modulePart) && string.IsNullOrWhiteSpace(body)) node.Title = "";
			else if (string.IsNullOrWhiteSpace(modulePart)) node.Title = body;
			else if (string.IsNullOrWhiteSpace(body)) node.Title = modulePart;
			else node.Title = $"{modulePart} · {body}";
		}

		moduleField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
		groupField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
		locationField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
		indexField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
		positionField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
		actionField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };
		kindField.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) UpdateTitle(); };

		UpdateTitle();

		kindField.PropertyChanged += async (_, e) =>
		{
			if (e.PropertyName != nameof(TreeItemVm.Value)) return;
			AddActiveChild();
			triggerModelChangedImmediate();
			_ = rebuildAsync();
		};

		targetsSection.Children.Add(node);
		targetsSection.IsExpanded = true;
		node.IsSelected = true;
	}

	private static TreeItemVm BuildSequencesSection(AutoDefinitionModel doc, AsyncRebuildDelegate rebuildAsync, TriggerSaveDelegate triggerModelChangedImmediate, FocusDelegate focusNewUnder, RenameDelegate renameSequenceAsync)
	{
		var sec = new TreeItemVm("sequences", TreeItemKind.Section) { HasAdd = true };
		sec.AddCommand = new RelayCommand(() =>
		{
			doc.Sequences.Add(new SequenceModel { Name = $"seq{doc.Sequences.Count + 1}", Events = new() { doc.Events.FirstOrDefault()?.Name ?? "" }, Start1 = new(), Start2 = new(), Start3 = new() });
			TreeHelper.EnsureSequenceBackingLists(doc);
			_ = rebuildAsync();
			triggerModelChangedImmediate();
			focusNewUnder("sequences", null);
		});

		foreach (var s in doc.Sequences)
		{
			var originalName = s.Name;
			var node = new TreeItemVm(s.Name, TreeItemKind.Object) { HasRemove = true, RemoveCommand = new RelayCommand(() => { doc.Sequences.Remove(s); _ = rebuildAsync(); triggerModelChangedImmediate(); }), Model = s, IsCaptionEditable = true };
			node.PropertyChanged += async (_, e) =>
			{
				if (e.PropertyName != nameof(TreeItemVm.Title)) return;
				var newName = node.Title?.Trim() ?? "";
				var oldName = originalName ?? "";
				if (string.IsNullOrWhiteSpace(newName)) { node.Title = oldName; return; }
				if (string.Equals(newName, oldName, StringComparison.OrdinalIgnoreCase)) return;
				if (doc.Sequences.Any(x => string.Equals(x.Name, newName, StringComparison.OrdinalIgnoreCase))) { node.Title = oldName; return; }
				_ = renameSequenceAsync(oldName, newName);
			};

			s.Events ??= new List<string>();
			s.Start1 ??= new List<string>();
			s.Start2 ??= new List<string>();
			s.Start3 ??= new List<string>();

			var eventOptions = doc.Events.Select(ev => ev.Name ?? "").Where(n => !string.IsNullOrWhiteSpace(n)).ToList();

			var eventsNode = new TreeItemVm("events", TreeItemKind.Object) { Model = s.Events, HasAdd = true };
			eventsNode.AddCommand = new RelayCommand(() =>
			{
				var defaultEvent = eventOptions.FirstOrDefault() ?? "";
				s.Events.Add(defaultEvent);
				_ = rebuildAsync();
				triggerModelChangedImmediate();
				focusNewUnder("sequences", s.Name);
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
					_ = rebuildAsync();
					triggerModelChangedImmediate();
				});
				eventsNode.Children.Add(field);
			}
			node.Children.Add(eventsNode);

			void AddStartArrayNode(string title, List<string> list)
			{
				var arrNode = new TreeItemVm(title, TreeItemKind.Object) { Model = list, HasAdd = true };
				arrNode.AddCommand = new RelayCommand(() =>
				{
					var defaultEvent = eventOptions.FirstOrDefault() ?? "";
					list.Add(defaultEvent);
					_ = rebuildAsync();
					triggerModelChangedImmediate();
					focusNewUnder("sequences", s.Name);
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
						_ = rebuildAsync();
						triggerModelChangedImmediate();
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

	// Simple helpers
	private static string GetSingularName(string plural)
	{
		if (string.IsNullOrWhiteSpace(plural)) return plural ?? "";
		if (plural.EndsWith("ies", StringComparison.OrdinalIgnoreCase) && plural.Length > 3)
			return plural.Substring(0, plural.Length - 3) + "y";
		if (plural.EndsWith("s", StringComparison.OrdinalIgnoreCase) && plural.Length > 1)
			return plural.Substring(0, plural.Length - 1);
		return plural;
	}
}
