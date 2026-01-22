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

	public static TreeItemVm CreateNumberOrParamField(string label, object model, Func<object> get, Action<object> set, IDictionary<string,double>? paramsMap)
	{
		var n = new TreeItemVm(label, TreeItemKind.Field) { IsCaptionEditable = false, Editor = TreeEditorKind.NumberOrParam, Model = model };
		var initial = get();
		// set initial value without marking dirty (preserve expressions as strings)
		n.SetInitialValue(initial);

		// if paramsMap provided, attempt to compute a calculated numeric value for display
		double? calc = null;
		if (paramsMap != null)
		{
			// Attempt to resolve numeric value without mutating model:
			if (initial is double d) calc = d;
			else if (initial is float f) calc = Convert.ToDouble(f);
			else if (initial is int i) calc = Convert.ToDouble(i);
			else if (initial is long l) calc = Convert.ToDouble(l);
			else if (initial is string s)
			{
				if (ParamAwareConverters.TryResolveParamString(s, paramsMap, out var resolved)) calc = resolved;
				else if (double.TryParse(s, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var parsed)) calc = parsed;
			}
			// arrays or lists are not collapsed here; you can extend to handle IList<double> etc.
		}

		// Add param keys/options if caller supplies them via model binding (number-or-param UI expects param names)
		if (paramsMap != null)
		{
			// insert a calculated hint as the first option if we have one
			if (calc.HasValue)
			{
				var hint = $"(= {calc.Value:G})";
				n.Options.Insert(0, hint);
			}
			// then populate remaining Options with param names (so the ComboBox still contains usable params)
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

	public static TreeItemVm CreateTranslationNode(TranslationModel tr, IEnumerable<string> paramKeys)
	{
		var node = new TreeItemVm("translation", TreeItemKind.Object) { Model = tr };
		tr.Position ??= new() { 0d, 0d, 0d };
		while (tr.Position.Count < 3) tr.Position.Add(0d);

		node.Children.Add(CreateNumberOrParamField("position[0]", tr, () => tr.Position[0], v => tr.Position[0] = v, paramKeys.ToDictionary(k => k, k => 0d)));
		node.Children.Add(CreateNumberOrParamField("position[1]", tr, () => tr.Position[1], v => tr.Position[1] = v, paramKeys.ToDictionary(k => k, k => 0d)));
		node.Children.Add(CreateNumberOrParamField("position[2]", tr, () => tr.Position[2], v => tr.Position[2] = v, paramKeys.ToDictionary(k => k, k => 0d)));

		var pu = new TreeItemVm("positionUnits", TreeItemKind.Field) { Editor = TreeEditorKind.Enum, Value = tr.PositionUnits ?? "inches", Model = tr };
		pu.Options.Add("meters"); pu.Options.Add("inches");
		pu.PropertyChanged += (_, e) => { if (e.PropertyName == nameof(TreeItemVm.Value)) tr.PositionUnits = pu.Value?.ToString(); };
		node.Children.Add(pu);

		node.Children.Add(CreateNumberOrParamField("rotation", tr, () => tr.Rotation ?? 0d, v => tr.Rotation = v, paramKeys.ToDictionary(k => k, k => 0d)));

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