using System;
using System.Collections;
using System.Collections.ObjectModel;
using System.Windows.Input;

namespace AutoJsonBuilder;

public enum TreeItemKind { Root, Section, Object, Field }

// Add distinct editor kinds for int textbox + list dropdown
public enum TreeEditorKind { None, Text, Int, NumberOrParam, Enum, Bool, List }

public sealed class TreeItemVm : NotifyBase
{
    public TreeItemVm(string title, TreeItemKind kind = TreeItemKind.Object)
    {
        Id = Guid.NewGuid();
        _title = title;
        Kind = kind;
    }

    public Guid Id { get; }

    private string _title;
    public string Title { get => _title; set => Set(ref _title, value); }

    // Title shown in the tree (editable)
    public string Caption
    {
        get => Title;
        set => Title = value;
    }

    // If false, render as TextBlock (section headers); if true, render as TextBox
    public bool IsCaptionEditable { get; set; } = false;

    public TreeItemKind Kind { get; }

    public ObservableCollection<TreeItemVm> Children { get; } = new();

    public bool HasAdd { get; set; }
    public bool HasRemove { get; set; }

    public ICommand? AddCommand { get; set; }
    public ICommand? RemoveCommand { get; set; }

    public object? Model { get; set; }

    private bool _isExpanded;
    public bool IsExpanded { get => _isExpanded; set => Set(ref _isExpanded, value); }

    private bool _isSelected;
    public bool IsSelected { get => _isSelected; set => Set(ref _isSelected, value); }

    // Editable leaf support
    public TreeEditorKind Editor { get; set; } = TreeEditorKind.None;

    private object? _value;
    public object? Value { get => _value; set => Set(ref _value, value); }

    public ObservableCollection<string> Options { get; } = new(); // for ComboBox

    // If set, can regenerate Options when backing lists change
    public Func<IEnumerable<string>>? OptionsProvider { get; set; }

    public void RefreshOptions()
    {
        if (OptionsProvider is null) return;

        Options.Clear();
        foreach (var s in OptionsProvider().Where(x => !string.IsNullOrWhiteSpace(x)).Distinct())
            Options.Add(s);
    }

    public bool IsEditable => Editor != TreeEditorKind.None;

    // For dropdowns that should bind directly to underlying arrays (e.g., _doc.Groups)
    private IEnumerable? _itemsSource;
    public IEnumerable? ItemsSource { get => _itemsSource; set => Set(ref _itemsSource, value); }
}
