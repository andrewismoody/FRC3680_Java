using System;
using System.Collections.Generic;
using System.Linq;
using System.Globalization;
using AutoJsonBuilder;
using AutoJsonBuilder.Models;

namespace AutoJsonBuilder.Helpers;

internal static class TreeHelper
{
	// Callback assigned by VM to perform the actual save/prompt action.
	// The helpers below schedule/trigger that callback in a debounced way.
	public static Action? OnModelChangedCallback;

	// Debounce implementation: ScheduleModelChanged delays invocation until user stops typing.
	// TriggerModelChangedImmediate forces an immediate invocation (used for explicit add/remove/rename commands).
	private static readonly object _debounceLock = new();
	private static System.Threading.Timer? _debounceTimer;
	private const int DefaultDebounceMs = 700;

	public static void ScheduleModelChanged(int delayMs = DefaultDebounceMs)
	{
		// No-op: scheduling removed. The UI should commit edits on LostFocus and call
		// TreeHelper.TriggerModelChangedImmediate() when a save is desired.
		return;
	}

	public static void TriggerModelChangedImmediate()
	{
		lock (_debounceLock)
		{
			try
			{
				_debounceTimer?.Dispose();
				_debounceTimer = null;
			}
			catch { }
		}
		try { OnModelChangedCallback?.Invoke(); } catch { /* swallow */ }
	}

	public static void RefreshAllOptions(IEnumerable<TreeItemVm> roots)
	{
		if (roots is null) return;
		foreach (var root in roots)
			RefreshAllOptions(root);
	}

	private static void RefreshAllOptions(TreeItemVm node)
	{
		node.RefreshOptions();
		foreach (var c in node.Children)
			RefreshAllOptions(c);
	}

	public static void FillOptions(TreeItemVm node, IEnumerable<string> values)
	{
		node.Options.Clear();
		if (values is null) return;
		foreach (var v in values.Where(x => !string.IsNullOrWhiteSpace(x)).Distinct())
			node.Options.Add(v);
	}

	// --- New: factory helpers moved from VM ---

	public static TreeItemVm CreateField(string title, object model, Func<string> get, Action<string> set)
	{
		var node = new TreeItemVm(title, TreeItemKind.Field) { Editor = TreeEditorKind.Text, Model = model };
		node.SetInitialValue(get() ?? "");
		node.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName == nameof(TreeItemVm.Value) && node.Value is string s)
			{
				// update model immediately but do NOT trigger save here.
				// View should call TreeHelper.TriggerModelChangedImmediate() on LostFocus.
				set(s);
			}
		};
		return node;
	}

	public static TreeItemVm CreateTextField(string label, object model, Func<string> get, Action<string> set)
		=> CreateField(label, model, get, set);

	// updated signature: paramsMap can contain heterogeneous values (double, string, IEnumerable<object>)
	public static TreeItemVm CreateNumberOrParamField(string label, object model, Func<object> get, Action<object> set, IDictionary<string, object>? paramsMap, bool captionEditable = false)
	{
		var valPrefix = "Evaluated Value: ";
		var n = new TreeItemVm(label, TreeItemKind.Field) { IsCaptionEditable = captionEditable, Editor = TreeEditorKind.NumberOrParam, Model = model };
		var initial = get();
		// set initial value without marking dirty (preserve expressions as strings)
		n.SetInitialValue(initial);

		// compute a human-friendly calculated preview (scalar or vector) from paramsMap if available
		string? hint = null;

		// existing preview logic (keeps working for non-param cases)
		if (paramsMap != null)
		{
			// Helper to attempt resolving a single element (object may be double/string or param ref)
			bool TryResolveElement(object? el, out double resolved)
			{
				resolved = 0d;
				if (el is double dd) { resolved = dd; return true; }
				if (el is float f) { resolved = Convert.ToDouble(f); return true; }
				if (el is int i) { resolved = Convert.ToDouble(i); return true; }
				if (el is long l) { resolved = Convert.ToDouble(l); return true; }
				if (el is string s)
				{
					// try direct numeric parse
					if (double.TryParse(s, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var pd)) { resolved = pd; return true; }
					// try param expression resolution
					if (ParamAwareConverters.TryResolveParamString(s, paramsMap, out var rv)) { resolved = rv; return true; }
				}
				return false;
			}

            // scalar initial
            if (initial is double sd) { hint = $"{valPrefix}{sd:G}"; }
			else if (initial is string strInit)
			{
				if (TryResolveElement(strInit, out var rv)) hint = $"{valPrefix}{rv:G}";
			}
			else if (initial is IEnumerable<object> seq)
			{
				// attempt to resolve each element into numbers for a vector preview
				var resolvedList = new List<string>();
				foreach (var e in seq)
				{
					if (TryResolveElement(e, out var rv)) resolvedList.Add(rv.ToString("G"));
					else resolvedList.Add("?");
				}
				hint = valPrefix + "[" + string.Join(", ", resolvedList) + "]";
			}
			else if (initial is System.Collections.IEnumerable seq2 && !(initial is string))
			{
				var resolvedList = new List<string>();
				foreach (var e in seq2)
				{
					if (TryResolveElement(e, out var rv)) resolvedList.Add(rv.ToString("G"));
					else resolvedList.Add("?");
				}
				hint = valPrefix + "[" + string.Join(", ", resolvedList) + "]";
			}
		}

		 // If this field is likely an expression/param (string) we replace combobox with a plain text editor
		// and surface a tooltip with the evaluated value. We also keep the old hint behavior for numeric cases.
		bool isParamLike = initial is string || (initial is null && paramsMap != null);

		if (isParamLike)
		{
			// replace editor with text box (no options)
			n.Editor = TreeEditorKind.Text;
			n.Options.Clear();

			// helper to update tooltip/preview child
			void SetPreview(string? text)
			{
				// compute evaluated value
				double val = 0.0;
				var resolved = false;
				if (!string.IsNullOrWhiteSpace(text) && paramsMap != null)
					resolved = ParamAwareConverters.TryResolveParamString(text, paramsMap, out val);

				string preview = resolved ? $"{valPrefix}{val:G}" : valPrefix + "(n/a)";
				// try set ToolTip property if present
				var prop = n.GetType().GetProperty("ToolTip");
				if (prop != null && prop.PropertyType == typeof(string))
				{
					try { prop.SetValue(n, preview); }
					catch { /* ignore */ }
				}
				else
				{
					// fallback: ensure a single readonly child with the preview text
					// remove any existing preview child (Title starts with "(=")
					var existing = n.Children.FirstOrDefault(c => c.Title != null && c.Title.StartsWith("(="));
					if (existing != null)
					{
						existing.Title = preview;
					}
					else
					{
						var previewNode = new TreeItemVm(preview, TreeItemKind.Field) { Editor = TreeEditorKind.None, Model = null };
						previewNode.IsCaptionEditable = false;
						previewNode.HasRemove = false;
						// insert preview as first child so it's visible
						n.Children.Insert(0, previewNode);
					}
				}
			}

			// set initial preview based on initial value (if string)
			SetPreview(initial as string);

			// on Value change, persist new string into model via set, and refresh preview
			n.PropertyChanged += (_, e) =>
			{
				if (e.PropertyName == nameof(TreeItemVm.Value))
				{
					var v = n.Value;
					// write back raw string/value to model
					set(v ?? "");
					// re-evaluate preview if text
					var s = v?.ToString() ?? "";
					SetPreview(s);
					// do NOT trigger save here; UI should call TriggerModelChangedImmediate on LostFocus
				}
			};

			// done: return field configured as text with preview tooltip/child
			return n;
		}

		// Non-param case: keep previous behavior (preview as first Options entry + param names)
		if (!string.IsNullOrWhiteSpace(hint))
			n.Options.Insert(0, hint);

		if (paramsMap != null)
		{
			foreach (var k in paramsMap.Keys) n.Options.Add(k);
		}

		// For non-editable NumberOrParam fields (options present) treat selection changes as immediate commits.
		n.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName != nameof(TreeItemVm.Value) || n.Value is null) return;
			set(n.Value);
			// selection/change -> immediate save
			TriggerModelChangedImmediate();
		};
		return n;
	}

	public static TreeItemVm CreateListField(string label, object model, Func<string> get, Action<string> set, IList<string> source)
	{
		var n = new TreeItemVm(label, TreeItemKind.Field) { Editor = TreeEditorKind.List, Model = model, OptionsProvider = () => source };
		n.SetInitialValue(get());
		n.RefreshOptions();
		n.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName == nameof(TreeItemVm.Value) && n.Value is string s && !string.IsNullOrWhiteSpace(s))
			{
				set(s);
				 // list selection -> immediate save
				TriggerModelChangedImmediate();
			}
		};
		return n;
	}

	public static TreeItemVm CreateListField(string label, object model, Func<string> get, Action<string> set, Func<IEnumerable<string>> optionsProvider)
	{
		var n = new TreeItemVm(label, TreeItemKind.Field) { IsCaptionEditable = false, Editor = TreeEditorKind.List, Model = model, OptionsProvider = optionsProvider };
		n.RefreshOptions();
		n.SetInitialValue(get());
		n.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName == nameof(TreeItemVm.Value) && n.Value is string s && !string.IsNullOrWhiteSpace(s))
			{
				set(s);
				 // selection -> immediate save
				TriggerModelChangedImmediate();
			}
		};
		return n;
	}

	public static TreeItemVm CreateIntField(string label, object model, Func<int> get, Action<int> set)
	{
		var n = new TreeItemVm(label, TreeItemKind.Field) { IsCaptionEditable = false, Editor = TreeEditorKind.Int, Model = model };
		n.SetInitialValue(get().ToString());
		n.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName != nameof(TreeItemVm.Value)) return;
			if (int.TryParse(n.Value?.ToString(), out var i))
			{
				set(i);
				 // integer change -> immediate save
				TriggerModelChangedImmediate();
			}
		};
		return n;
	}

	public static TreeItemVm CreatePoseDropdown(string label, object model, Func<string> get, Action<string> set, IList<string> source)
	{
		var n = new TreeItemVm(label, TreeItemKind.Field) { Editor = TreeEditorKind.List, Model = model };

		// Determine current model value and prefer "any" when empty.
		var curVal = get();
		var initial = string.IsNullOrWhiteSpace(curVal) ? "any" : curVal;

		// Populate options from source then ensure "any" is present as the primary default.
		FillOptions(n, source);
		if (!n.Options.Contains("any")) n.Options.Insert(0, "any");

		// If the model has a concrete current value that isn't in the options, preserve it just after "any".
		if (!string.IsNullOrWhiteSpace(curVal) && !n.Options.Contains(curVal))
			n.Options.Insert(1, curVal);

		// Set the UI initial selection (does not immediately mutate model).
		n.SetInitialValue(initial);

		n.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName == nameof(TreeItemVm.Value) && n.Value is string s && !string.IsNullOrWhiteSpace(s))
			{
				set(s);
				// dropdown selection -> immediate save
				TriggerModelChangedImmediate();
			}
		};
		return n;
	}

	public static TreeItemVm CreatePoseListField(string label, object model, Func<string> get, Action<string> set, Func<IEnumerable<string>> optionsProvider)
	{
		var n = new TreeItemVm(label, TreeItemKind.Field) { Editor = TreeEditorKind.List, Model = model, OptionsProvider = optionsProvider };

		// Refresh available options and ensure "any" default exists
		n.RefreshOptions();
		if (!n.Options.Contains("any")) n.Options.Insert(0, "any");

		// Preserve current model value if present (place it after "any" when missing)
		var curVal = get();
		if (!string.IsNullOrWhiteSpace(curVal) && !n.Options.Contains(curVal))
			n.Options.Insert(1, curVal);

		// Use "any" as the shown initial value when model value is empty
		var initial = string.IsNullOrWhiteSpace(curVal) ? "any" : curVal;
		n.SetInitialValue(initial);

		n.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName == nameof(TreeItemVm.Value) && n.Value is string s && !string.IsNullOrWhiteSpace(s))
			{
				set(s);
				// selection -> immediate save
				TriggerModelChangedImmediate();
			}
		};
		return n;
	}

	// Public: map array indices to axis labels: 0->X, 1->Y, 2->Z, otherwise numeric index.
	public static string GetArrayIndexLabel(int index) => index switch { 0 => "X", 1 => "Y", 2 => "Z", _ => index.ToString() };

	// Public: normalize any title string by replacing bracketed numeric indices with axis labels.
	// Example: "position[0]" => "position[X]" and "[0]" => "[X]". Use when building tree node titles for params/arrays.
	public static string NormalizeIndexLabels(string? s)
	{
		if (string.IsNullOrEmpty(s)) return s ?? string.Empty;
		// Only replace the first three numeric bracket tokens; lightweight and safe for existing strings.
		return s.Replace("[0]", "[X]").Replace("[1]", "[Y]").Replace("[2]", "[Z]");
	}

	public static TreeItemVm CreateTranslationNode(TranslationModel tr, IEnumerable<string> paramKeys)
	{
		var node = new TreeItemVm("translation", TreeItemKind.Object) { Model = tr };

		// Ensure position is present and exactly 3 elements.
		tr.Position ??= new() { 0d, 0d, 0d };
		while (tr.Position.Count < 3) tr.Position.Add(0d);

		// Create a parent "position" node so it can be expanded/collapsed like params arrays.
		var posNode = new TreeItemVm("position", TreeItemKind.Object) { Model = tr.Position };
		for (var idx = 0; idx < 3; idx++)
		{
			var index = idx; // capture for lambda
			// child labeled X/Y/Z and bound to tr.Position[index]
			var child = CreateNumberOrParamField(GetArrayIndexLabel(index), tr, () => tr.Position[index], v => tr.Position[index] = v, paramKeys.ToDictionary(k => k, k => (object)0d));
			// Render position elements the same way param array elements do: plain text editors (no Options).
			child.Editor = TreeEditorKind.Text;
			child.Options.Clear();
			posNode.Children.Add(child);
		}
		node.Children.Add(posNode);

		var pu = new TreeItemVm("positionUnits", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = tr.PositionUnits ?? "inches", Model = tr };
		pu.Options.Add("meters"); pu.Options.Add("inches");
		pu.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) tr.PositionUnits = pu.Value?.ToString(); };
		node.Children.Add(pu);

		// Rotation should be a plain text editor (accept numeric literal or param expression).
		var rot = CreateNumberOrParamField("rotation", tr, () => tr.Rotation ?? 0d, v => tr.Rotation = v, paramKeys.ToDictionary(k => k, k => (object)0d));
		rot.Editor = TreeEditorKind.Text;
		rot.Options.Clear();
		node.Children.Add(rot);

		var ru = new TreeItemVm("rotationUnits", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = tr.RotationUnits ?? "degrees", Model = tr };
		ru.Options.Add("radians"); ru.Options.Add("degrees");
		ru.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) tr.RotationUnits = ru.Value?.ToString(); };
		node.Children.Add(ru);

		return node;
	}

	// --- New: traversal helpers moved from VM ---

	public static void FocusNewUnder(IEnumerable<TreeItemVm> roots, string sectionTitle, string? childTitle = null)
	{
		if (roots == null) return;
		var root = roots.FirstOrDefault();
		if (root is null) return;
		var sec = root.Children.FirstOrDefault(x => x.Title == sectionTitle);
		if (sec is null) return;

		sec.IsExpanded = true;

		var child = childTitle is null
			? sec.Children.LastOrDefault()
			: sec.Children.LastOrDefault(x => x.Title == childTitle) ?? sec.Children.LastOrDefault();

		if (child is null) return;
		child.IsSelected = true;
	}

	public static TreeItemVm? FindSection(IEnumerable<TreeItemVm> roots, string title)
	{
		var root = roots.FirstOrDefault();
		return root?.Children.FirstOrDefault(c => c.Title == title);
	}

	// Simple container for saved view state
	public sealed class TreeViewState
	{
		public HashSet<string> Expanded { get; } = new(StringComparer.Ordinal);
		public string? SelectedPath { get; set; }
	}

	// Build a path for a node by concatenating titles with '/' (root-first).
	private static string BuildPath(string parentPath, TreeItemVm node)
		=> string.IsNullOrEmpty(parentPath) ? (node.Title ?? "") : (parentPath + "/" + (node.Title ?? ""));

	// Capture expanded nodes and selected node path for all roots.
	public static TreeViewState GetViewState(IEnumerable<TreeItemVm> roots)
	{
		var state = new TreeViewState();
		if (roots == null) return state;

		// Start collection at first-level children under each root so the saved paths are
		// independent of the root title (which changes when the file is saved / renamed).
		void collect(TreeItemVm node, string path)
		{
			var cur = BuildPath(path, node);
			if (node.IsExpanded) state.Expanded.Add(cur);
			if (node.IsSelected) state.SelectedPath = cur;
			foreach (var c in node.Children) collect(c, cur);
		}

		foreach (var r in roots)
		{
			// iterate children so stored paths are root-agnostic
			foreach (var c in r.Children) collect(c, "");
		}

		return state;
	}

	// Apply saved state to the newly built tree. Returns the node that was selected (if found).
	public static TreeItemVm? ApplyViewState(IEnumerable<TreeItemVm> roots, TreeViewState state)
	{
		if (roots == null || state == null) return null;
		TreeItemVm? selected = null;

		// Apply state to first-level children under each root (saved state is root-agnostic).
		void apply(TreeItemVm node, string path)
		{
			var cur = BuildPath(path, node);
			node.IsExpanded = state.Expanded.Contains(cur);
			if (state.SelectedPath != null && state.SelectedPath == cur)
			{
				node.IsSelected = true;
				selected = node;
			}
			else
			{
				node.IsSelected = false;
			}
			foreach (var c in node.Children) apply(c, cur);
		}

		foreach (var r in roots)
		{
			foreach (var c in r.Children) apply(c, "");
		}

		return selected;
	}

	public static TreeItemVm CreateFixtureRefNode(string title, object model /* FixtureRefModel expected */, Func<string> getType, Action<string> setType, Func<int> getIndex, Action<int> setIndex, Func<IEnumerable<string>>? optionsProvider = null)
	{
		// fixtureref label must NOT be editable in the tree
		// If optionsProvider is supplied, render a single dropdown (reference) that selects an existing fixture "type:index".
		if (optionsProvider != null)
		{
			var list = CreateListField(title, model,
				() => $"{getType()}:{getIndex()}",
				(v) =>
				{
					if (string.IsNullOrWhiteSpace(v)) return;
					var parts = v.Split(':');
					if (parts.Length >= 2 && int.TryParse(parts[^1], out var idx))
					{
						var type = string.Join(":", parts.Take(parts.Length - 1));
						setType(type);
						setIndex(idx);
					}
					else
					{
						// fallback: try to match exactly
						var found = optionsProvider().FirstOrDefault(x => x == v);
						if (found != null)
						{
							var p = found.Split(':');
							if (p.Length >= 2 && int.TryParse(p[^1], out var i2))
							{
								setType(string.Join(":", p.Take(p.Length - 1)));
								setIndex(i2);
							}
						}
					}
				},
				optionsProvider);
			list.IsCaptionEditable = false;
			list.Model = model;
			return list;
		}

		// fallback: separate type + index fields (used when no global fixture list is available)
		var node = new TreeItemVm(title, TreeItemKind.Object) { Model = model, IsCaptionEditable = false };
		// type
		var t = CreateTextField("type", model, getType, setType);
		node.Children.Add(t);
		// index
		var idxField = CreateIntField("index", model, getIndex, setIndex);
		node.Children.Add(idxField);
		return node;
	}

	// Create a derivedFrom node editor that matches derivedFrom.schema.json
	// fixtureOptionsProvider: optional provider that returns "type:index" strings for existing fixtures.
	public static TreeItemVm CreateDerivedFromNode(object model /* DerivedFromModel expected */, IEnumerable<string> paramKeys, Func<IEnumerable<string>>? fixtureOptionsProvider = null, Func<object?>? getOffset = null, Action<object?>? setOffset = null)
	{
		// The model parameter is left as object to avoid tight coupling; callers set Model appropriately.
		// derivedFrom label must NOT be editable
		var node = new TreeItemVm("derivedFrom", TreeItemKind.Object) { Model = model, IsCaptionEditable = false };

		// function enum
		var fn = new TreeItemVm("function", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = GetPropAsString(model, "Function") ?? "none", Model = model };
		fn.Options.Add("parallel"); fn.Options.Add("perpendicular"); fn.Options.Add("bisector"); fn.Options.Add("none");
		fn.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName == nameof(TreeItemVm.Value))
			{
				SetProp(model, "Function", fn.Value?.ToString());
				TriggerModelChangedImmediate();
			}
		};
		node.Children.Add(fn);

		 // For each slot pair (primary and secondary) provide a small wrapper that lets the user
		// choose between a FixtureRef or a nested DerivedFrom. This mirrors the schema's "oneOf"
		// semantics for fixture/derivedFrom and fixture2/derivedFrom2, and allows recursion.
		void AddSlot(string slotBaseName) // slotBaseName: "" or "2"
		{
			var fixtureProp = "Fixture" + (slotBaseName == "" ? "" : "2");
			var derivedProp = "DerivedFrom" + (slotBaseName == "" ? "" : "2");
			var slotTitle = slotBaseName == "" ? "slot" : "slot2";
		
			// wrapper node to host a small kind selector plus the actual child editor
			// slot labels must NOT be editable
			var slotNode = new TreeItemVm(slotTitle, TreeItemKind.Object) { Model = model, IsCaptionEditable = false };
		
			// determine current kind based on which prop is present on the model
			var fixtureObj = GetProp(model, fixtureProp);
			var derivedObj = GetProp(model, derivedProp);
			var initialKind = fixtureObj != null ? "fixture" : (derivedObj != null ? "derivedFrom" : "fixture");
		
			// small selector to choose which kind to edit
			var kindField = new TreeItemVm("kind", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = initialKind, Model = model };
			kindField.Options.Add("fixture"); kindField.Options.Add("derivedFrom");
			slotNode.Children.Add(kindField);
		
			// helper to create the active child under the slot
			void CreateChildForKind(string kind)
			{
				// remove any existing child named "fixture"/"derivedFrom"
				var existing = slotNode.Children.FirstOrDefault(c => c.Title == "fixture" || c.Title == "derivedFrom");
				if (existing != null) slotNode.Children.Remove(existing);
		
				if (kind == "fixture")
				{
					// ensure Fixture model exists
					var fObj = GetProp(model, fixtureProp);
					if (fObj == null)
					{
						var frcType = GetTypeByName("FixtureRefModel");
						if (frcType != null) fObj = Activator.CreateInstance(frcType);
						SetProp(model, fixtureProp, fObj);
					}
					var cur = GetProp(model, fixtureProp);
					if (cur != null)
					{
						 // Pass the fixtureOptionsProvider so fixture references render as a dropdown of known fixtures.
						slotNode.Children.Add(CreateFixtureRefNode("fixture", cur,
							() => GetPropAsString(cur, "Type") ?? "",
							v => SetProp(cur, "Type", v),
							() => GetPropAsInt(cur, "Index"),
							v => SetProp(cur, "Index", v),
							fixtureOptionsProvider));
					}
					// remove any derivedFrom model to respect oneOf
					SetProp(model, derivedProp, null);
				}
				else // derivedFrom
				{
					// ensure DerivedFrom model exists
					var dObj = GetProp(model, derivedProp);
					if (dObj == null)
					{
						var dfType = GetTypeByName("DerivedFromModel");
						if (dfType != null) dObj = Activator.CreateInstance(dfType);
						SetProp(model, derivedProp, dObj);
					}
					var cur = GetProp(model, derivedProp);
					if (cur != null)
					{
						 // pass fixtureOptionsProvider recursively for nested derivedFrom nodes
						slotNode.Children.Add(CreateDerivedFromNode(cur, paramKeys, fixtureOptionsProvider));
					}
					// remove any fixture model to respect oneOf
					SetProp(model, fixtureProp, null);
				}
				TriggerModelChangedImmediate();
			}
		
			// initial child
			CreateChildForKind(kindField.Value?.ToString() ?? initialKind);
		
			// when user switches kind, swap child and model props
			kindField.PropertyChanged += (_, e) =>
			{
				if (e.PropertyName != nameof(TreeItemVm.Value)) return;
				var k = kindField.Value?.ToString() ?? "fixture";
				CreateChildForKind(k);
			};
		
			// expose the slot node as either "fixture"/"fixture2" if it currently contains a fixture,
			// otherwise title remains "derivedFrom"/"derivedFrom2" depending on child. For simplicity
			// keep the wrapper title but the child nodes are correctly named for schema paths.
			node.Children.Add(slotNode);
		}
		
		// add primary and secondary slots
		AddSlot("");
		AddSlot("2");
		
		// offset: allow either numeric literal or parameter-expression string.
		object? offVal = GetProp(model, "Offset");
		var offsetText = offVal == null ? "" : offVal.ToString() ?? "";
		var offsetField = new TreeItemVm("offset", TreeItemKind.Field) { Editor = TreeEditorKind.Text, Model = model };
		offsetField.SetInitialValue(offsetText);
		offsetField.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName != nameof(TreeItemVm.Value)) return;
			var s = offsetField.Value?.ToString() ?? "";
			// Prefer numeric storage when the user typed a plain number
			if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var d))
			{
				SetProp(model, "Offset", d);
			}
			else if (string.IsNullOrWhiteSpace(s))
			{
				// empty -> remove/zero (schema default is 0)
				SetProp(model, "Offset", 0d);
			}
			else
			{
				// keep as string (param expression)
				SetProp(model, "Offset", s);
			}
			TriggerModelChangedImmediate();
		};
		node.Children.Add(offsetField);
		
		return node;
	}

	// reflection helpers (lightweight, null-safe) to avoid adding model references in helper project
	private static object? GetProp(object? obj, string name)
	{
		if (obj == null) return null;
		var p = obj.GetType().GetProperty(name);
		return p == null ? null : p.GetValue(obj);
	}
	private static string? GetPropAsString(object? obj, string name) => GetProp(obj, name)?.ToString();
	private static int GetPropAsInt(object? obj, string name)
	{
		var v = GetProp(obj, name);
		if (v == null) return 0;
		try { return Convert.ToInt32(v); } catch { return 0; }
	}
	private static void SetProp(object? obj, string name, object? value)
	{
		if (obj == null) return;
		var p = obj.GetType().GetProperty(name);
		if (p == null) return;
		try { p.SetValue(obj, value); } catch { }
	}
	private static Type? GetTypeByName(string name)
	{
		// search loaded assembly types for the model name
		foreach (var a in AppDomain.CurrentDomain.GetAssemblies())
		{
			var t = a.GetTypes().FirstOrDefault(x => x.Name == name);
			if (t != null) return t;
		}
		return null;
	}

	// Ensure every SequenceModel in the document has non-null lists for Events, Start1, Start2, Start3.
	public static void EnsureSequenceBackingLists(AutoDefinitionModel doc)
	{
		if (doc == null) return;
		doc.Sequences ??= new List<SequenceModel>();
		foreach (var s in doc.Sequences)
		{
			if (s == null) continue;
			s.Events ??= new List<string>();
			s.Start1 ??= new List<string>();
			s.Start2 ??= new List<string>();
			s.Start3 ??= new List<string>();
		}
	}

	// Ensure a single SequenceModel's lists exist.
	public static void EnsureSequenceLists(SequenceModel s)
	{
		if (s == null) return;
		s.Events ??= new List<string>();
		s.Start1 ??= new List<string>();
		s.Start2 ??= new List<string>();
		s.Start3 ??= new List<string>();
	}
}