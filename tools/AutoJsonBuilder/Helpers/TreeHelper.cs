using System;
using System.Collections.Generic;
using System.Linq;
using AutoJsonBuilder;
using AutoJsonBuilder.Models;

namespace AutoJsonBuilder.Helpers;

internal static class TreeHelper
{
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
				set(s);
		};
		return node;
	}

	public static TreeItemVm CreateTextField(string label, object model, Func<string> get, Action<string> set)
		=> CreateField(label, model, get, set);

	// updated signature: paramsMap can contain heterogeneous values (double, string, IEnumerable<object>)
	public static TreeItemVm CreateNumberOrParamField(string label, object model, Func<object> get, Action<object> set, IDictionary<string, object>? paramsMap)
	{
		var n = new TreeItemVm(label, TreeItemKind.Field) { IsCaptionEditable = false, Editor = TreeEditorKind.NumberOrParam, Model = model };
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
			if (initial is double sd) { hint = $"(= {sd:G})"; }
			else if (initial is string strInit)
			{
				if (TryResolveElement(strInit, out var rv)) hint = $"(= {rv:G})";
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
				hint = "(= [" + string.Join(", ", resolvedList) + "])";
			}
			else if (initial is System.Collections.IEnumerable seq2 && !(initial is string))
			{
				var resolvedList = new List<string>();
				foreach (var e in seq2)
				{
					if (TryResolveElement(e, out var rv)) resolvedList.Add(rv.ToString("G"));
					else resolvedList.Add("?");
				}
				hint = "(= [" + string.Join(", ", resolvedList) + "])";
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

				string preview = resolved ? $"(= {val:G})" : "(= ?)";
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

		n.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName == nameof(TreeItemVm.Value) && n.Value is not null)
				set(n.Value);
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
				set(s);
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
				set(s);
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
			if (int.TryParse(n.Value?.ToString(), out var i)) set(i);
		};
		return n;
	}

	public static TreeItemVm CreatePoseDropdown(string label, object model, Func<string> get, Action<string> set, IList<string> source)
	{
		var n = new TreeItemVm(label, TreeItemKind.Field) { Editor = TreeEditorKind.List, Model = model };
		n.SetInitialValue(get());
		FillOptions(n, source);
		if (n.Value is string cur && !string.IsNullOrWhiteSpace(cur) && !n.Options.Contains(cur))
			n.Options.Insert(0, cur);
		n.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName == nameof(TreeItemVm.Value) && n.Value is string s && !string.IsNullOrWhiteSpace(s))
				set(s);
		};
		return n;
	}

	public static TreeItemVm CreatePoseListField(string label, object model, Func<string> get, Action<string> set, Func<IEnumerable<string>> optionsProvider)
	{
		var n = new TreeItemVm(label, TreeItemKind.Field) { Editor = TreeEditorKind.List, Model = model, OptionsProvider = optionsProvider };
		n.RefreshOptions();
		n.SetInitialValue(get());
		if (n.Value is string cur && !string.IsNullOrWhiteSpace(cur) && !n.Options.Contains(cur))
			n.Options.Insert(0, cur);
		n.PropertyChanged += (_, e) =>
		{
			if (e.PropertyName == nameof(TreeItemVm.Value) && n.Value is string s && !string.IsNullOrWhiteSpace(s))
				set(s);
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
		tr.Position ??= new() { 0d, 0d, 0d };
		while (tr.Position.Count < 3) tr.Position.Add(0d);

		// Display axis labels instead of numeric indices (use shared helper)
		node.Children.Add(CreateNumberOrParamField($"position[{GetArrayIndexLabel(0)}]", tr, () => tr.Position[0], v => tr.Position[0] = v, paramKeys.ToDictionary(k => k, k => (object)0d)));
		node.Children.Add(CreateNumberOrParamField($"position[{GetArrayIndexLabel(1)}]", tr, () => tr.Position[1], v => tr.Position[1] = v, paramKeys.ToDictionary(k => k, k => (object)0d)));
		node.Children.Add(CreateNumberOrParamField($"position[{GetArrayIndexLabel(2)}]", tr, () => tr.Position[2], v => tr.Position[2] = v, paramKeys.ToDictionary(k => k, k => (object)0d)));

		var pu = new TreeItemVm("positionUnits", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = tr.PositionUnits ?? "inches", Model = tr };
		pu.Options.Add("meters"); pu.Options.Add("inches");
		pu.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) tr.PositionUnits = pu.Value?.ToString(); };
		node.Children.Add(pu);

		node.Children.Add(CreateNumberOrParamField("rotation", tr, () => tr.Rotation ?? 0d, v => tr.Rotation = v, paramKeys.ToDictionary(k => k, k => (object)0d)));

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
}