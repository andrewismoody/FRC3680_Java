using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using Microsoft.Win32;
using System.Windows.Data; // for Binding.TargetUpdatedEvent
using System.Windows.Media; // added for VisualTreeHelper

namespace AutoJsonBuilder;

// NOTE: ensure MainWindow.xaml has: xmlns:local="clr-namespace:AutoJsonBuilder"

public partial class MainWindow : Window
{
    private readonly MainWindowViewModel _vm = new();

    public MainWindow()
    {
        InitializeComponent();
        DataContext = _vm;

        CommandBindings.Add(new CommandBinding(ApplicationCommands.Close, (_, __) => Close()));

        // Listen for lost keyboard focus anywhere in this window (catches TextBoxes inside DataTemplates/Setters)
        AddHandler(Keyboard.LostKeyboardFocusEvent, new KeyboardFocusChangedEventHandler(Editor_LostKeyboardFocus), handledEventsToo: true);

        // Track when a TextBox gains keyboard focus so we can compare before/after text
        AddHandler(Keyboard.GotKeyboardFocusEvent, new KeyboardFocusChangedEventHandler(Editor_GotKeyboardFocus), handledEventsToo: true);

        // Listen for binding target updates (fires when NotifyOnTargetUpdated=True)
        AddHandler(Binding.TargetUpdatedEvent, new RoutedEventHandler(OnBindingTargetUpdated), handledEventsToo: true);
    }

    private void TreeView_SelectedItemChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
    {
        _vm.SetSelectedTreeItem(e.NewValue as TreeItemVm);
    }

    private async void Editor_LostFocus(object? sender, RoutedEventArgs e)
    {
        if (DataContext is not MainWindowViewModel vm) return;

        try
        {
            // If already saved -> write current doc immediate
            if (!string.IsNullOrWhiteSpace(vm.CurrentJsonPath))
            {
                await vm.SaveToPathAsync(vm.CurrentJsonPath);
                return;
            }

            // Not saved yet -> ensure Season exists
            if (string.IsNullOrWhiteSpace(vm.Season))
            {
                var season = PromptForSeason();
                if (string.IsNullOrWhiteSpace(season))
                    return; // user cancelled / didn't provide -> don't auto-save
                vm.Season = season;
            }

            // Suggest filename auto{season}.json
            var suggested = $"auto{vm.Season}.json";
            var dlg = new SaveFileDialog
            {
                Title = "Save auto JSON",
                Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*",
                FileName = suggested,
                OverwritePrompt = true,
                InitialDirectory = vm.GetDefaultDeployFolder() ?? Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments)
            };

            if (dlg.ShowDialog() == true)
            {
                await vm.SaveToPathAsync(dlg.FileName);
            }
        }
        catch (Exception ex)
        {
            // basic feedback and log in VM if available
            vm.Log($"Save on edit failed: {ex.Message}");
            MessageBox.Show(this, $"Failed to save edits: {ex.Message}", "Save error", MessageBoxButton.OK, MessageBoxImage.Error);
        }
    }

    // store initial text when a TextBox gains keyboard focus (only for tree items)
    private void Editor_GotKeyboardFocus(object? sender, KeyboardFocusChangedEventArgs e)
    {
        if (e.OriginalSource is not TextBox tb) return;

        // Log what control got focus and its DataContext/text
        var dc = tb.DataContext;
        _vm.Log($"GotFocus: Element=TextBox DataContext={(dc?.GetType().Name ?? "(null)")} Text='{tb.Text}'");

        // If it's a TreeItemVm, log the node's Title/Value/Editor/IsDirty for diagnostics
        if (dc is TreeItemVm node)
        {
            var nodeVal = node.Value?.ToString() ?? "(null)";
            _vm.Log($"  Node: Title='{node.Title}' Editor={node.Editor} Value='{nodeVal}' IsDirty={node.IsDirty}");
        }

        // NEW: walk visual tree upward to find nearest ancestor with a non-null DataContext
        DependencyObject current = tb;
        bool found = false;
        while (true)
        {
            var parent = VisualTreeHelper.GetParent(current);
            if (parent is null) break;

            if (parent is FrameworkElement fe && fe.DataContext is not null)
            {
                var ctxType = fe.DataContext.GetType().Name;
                _vm.Log($"AncestorFound: AncestorType={fe.GetType().Name} DataContextType={ctxType}");

                if (fe.DataContext is TreeItemVm ancestorNode)
                {
                    var ancVal = ancestorNode.Value?.ToString() ?? "(null)";
                    _vm.Log($"  AncestorNode: Title='{ancestorNode.Title}' Editor={ancestorNode.Editor} Value='{ancVal}' IsDirty={ancestorNode.IsDirty}");
                }

                found = true;
                break;
            }

            current = parent;
        }

        if (!found)
            _vm.Log("AncestorFound: none with non-null DataContext in visual tree.");

        // stash initial text into Tag (safe, local UI usage)
        tb.Tag = tb.Text ?? "";
    }

    private void Editor_LostKeyboardFocus(object? sender, KeyboardFocusChangedEventArgs e)
    {
        // Only react when the focus left a TextBox (avoid unrelated focus changes)
        if (e.OriginalSource is not TextBox tb) return;

        // Only handle TextBoxes that belong to the tree (bound to a TreeItemVm)
        if (tb.DataContext is not TreeItemVm node) return;

        // If initial text matches current text, nothing changed -> skip save
        var initial = tb.Tag as string;
        var current = tb.Text ?? "";
        if (initial == current) return;

        // mark node dirty (in case VM logic relies on it)
        node.IsDirty = true;

        // reuse existing handler (it is async void and accepts RoutedEventArgs)
        Editor_LostFocus(e.OriginalSource, e);

        // after save/rebuild the node will be replaced; no need to manually clear IsDirty here
    }

    private void OnBindingTargetUpdated(object? sender, RoutedEventArgs e)
    {
        if (sender is not FrameworkElement fe) return;

        // Only log bindings that come from our tree (have TreeItemVm DataContext)
        var dc = fe.DataContext;
        if (dc is null) return;

        var ctxType = dc.GetType().Name;

        string valueStr;
        switch (fe)
        {
            case TextBox tb:
                valueStr = tb.Text ?? "";
                break;
            case ComboBox cb:
                valueStr = cb.SelectedItem?.ToString() ?? (cb.Text ?? "");
                break;
            default:
                // try to read a 'Value' DP via reflection if present
                var prop = fe.GetType().GetProperty("Text") ?? fe.GetType().GetProperty("SelectedItem");
                valueStr = prop?.GetValue(fe)?.ToString() ?? fe.ToString();
                break;
        }

        _vm.Log($"Binding.TargetUpdated: Element={fe.GetType().Name} DataContext={ctxType} Value='{valueStr}'");
    }

    private string? PromptForSeason()
    {
        // small modal window asking for season text
        var win = new Window
        {
            Title = "Enter season",
            Width = 360,
            Height = 140,
            WindowStartupLocation = WindowStartupLocation.CenterOwner,
            Owner = this,
            ResizeMode = ResizeMode.NoResize
        };

        var panel = new StackPanel { Margin = new Thickness(10) };
        panel.Children.Add(new TextBlock { Text = "Please enter the season (used as filename prefix):", Margin = new Thickness(0,0,0,8) });

        var input = new TextBox { MinWidth = 300 };
        panel.Children.Add(input);

        var buttons = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Right, Margin = new Thickness(0,10,0,0) };
        var ok = new Button { Content = "OK", IsDefault = true, MinWidth = 80, Margin = new Thickness(0,0,8,0) };
        var cancel = new Button { Content = "Cancel", IsCancel = true, MinWidth = 80 };
        buttons.Children.Add(ok);
        buttons.Children.Add(cancel);
        panel.Children.Add(buttons);

        win.Content = panel;

        string? result = null;
        ok.Click += (_, __) => { result = input.Text?.Trim(); win.DialogResult = true; win.Close(); };
        cancel.Click += (_, __) => { win.DialogResult = false; win.Close(); };

        var dlg = win.ShowDialog();
        return dlg == true ? result : null;
    }
}
