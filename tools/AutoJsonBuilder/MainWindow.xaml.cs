using System;
using System.Windows;
using System.Windows.Controls; // <-- ensure TextBox type available
using System.Windows.Input;
using Microsoft.Win32;
using System.Windows.Data; // for Binding.TargetUpdatedEvent
using System.Windows.Media;
using AutoJsonBuilder.Helpers; // added for VisualTreeHelper
using System.Collections.Generic;
using System.Threading.Tasks;

namespace AutoJsonBuilder;

// NOTE: ensure MainWindow.xaml has: xmlns:local="clr-namespace:AutoJsonBuilder"

public partial class MainWindow : Window
{
    private readonly MainWindowViewModel _vm = new();

    // Tracks caption TextBoxes that are actively committing a pose rename so the global
    // LostKeyboardFocus handler can skip its save prompt and avoid a double prompt.
    private readonly HashSet<TextBox> _captionHandled = new();
    private readonly object _captionHandledLock = new();

    // Remember last observed binding value per editor control so we ignore the initial
    // target-update that fires when controls are (re)bound. Only trigger save when the
    // observed value actually changes.
    private readonly Dictionary<FrameworkElement, object?> _lastBindingValues = new();
    private readonly object _lastBindingValuesLock = new();

    public MainWindow()
    {
        InitializeComponent();
        DataContext = _vm;

        // QUICK WARM-UP: force WPF/DirectWrite/D3D init and warm any lazy JIT/resource work.
        // Keeps UI hidden work cheap; helps avoid many small pauses during the first visible rebuild.
        _ = Task.Run(() =>
        {
            try
            {
                // 1) Force a tiny render on UI thread to warm DirectX/text rendering paths.
                if (Application.Current?.Dispatcher != null)
                {
                    Application.Current.Dispatcher.Invoke(() =>
                    {
                        try
                        {
                            var dv = new System.Windows.Media.DrawingVisual();
                            var rtb = new System.Windows.Media.Imaging.RenderTargetBitmap(1, 1, 96, 96, System.Windows.Media.PixelFormats.Pbgra32);
                            rtb.Render(dv);
                        }
                        catch { /* best-effort */ }
                    });
                }

                // 2) Warm VM tree-building on a background thread (does not touch UI collections).
                try { _ = _vm.RebuildTreeAsync(); } catch { }
            }
            catch { /* swallow */ }
        });

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

    // Caption TextBox LostFocus: commit pose caption edits (VM handles validation/rename).
    private async void TreeItemCaption_LostFocus(object? sender, RoutedEventArgs e)
    {
        if (sender is not TextBox tb) return;
        if (tb.DataContext is not TreeItemVm item) return;
        if (DataContext is not MainWindowViewModel vm) return;

        // Mark this TextBox as handled so the global LostKeyboardFocus handler will skip prompting.
        lock (_captionHandledLock)
        {
            _captionHandled.Add(tb);
        }

        try
        {
            await vm.CommitPoseCaptionAsync(item);
        }
        catch
        {
            // swallow to avoid crashing UI; VM logs errors
        }
        finally
        {
            // Ensure the mark is removed eventually if the global handler didn't see it.
            _ = Task.Run(async () =>
            {
                await Task.Delay(1500).ConfigureAwait(false);
                lock (_captionHandledLock)
                {
                    _captionHandled.Remove(tb);
                }
            });
        }
    }

    // Optional: remember original text when focusing an editor (not strictly required).
    private void Editor_GotKeyboardFocus(object? sender, KeyboardFocusChangedEventArgs e)
    {
        // no-op; placeholder if you want to track original value for undo/compare
    }

    // Binding target updated: can be used to react when a ComboBox/TextBox binding propagates value.
    private void OnBindingTargetUpdated(object? sender, RoutedEventArgs e)
    {
        // Only trigger for Field nodes to avoid duplicate saves for structural changes.
        try
        {
            if (DataContext is not MainWindowViewModel vm) return;
            if (e.OriginalSource is not FrameworkElement fe) return;
            if (fe.DataContext is not TreeItemVm tivm) return;
            if (tivm.Kind != TreeItemKind.Field) return;

            // Use the TreeItemVm.Value as the canonical value (may be string, bool, int, ...)
            var current = tivm.Value;

            lock (_lastBindingValuesLock)
            {
                if (!_lastBindingValues.TryGetValue(fe, out var prev))
                {
                    // First time we see this editor instance: record and do not trigger a save.
                    _lastBindingValues[fe] = CloneForComparison(current);
                    return;
                }

                // Compare using object.Equals on the canonical values (null-safe).
                if (AreEqualForSave(prev, current))
                {
                    // No real change — ignore.
                    return;
                }

                // Value changed: update stored value and trigger save.
                _lastBindingValues[fe] = CloneForComparison(current);
            }

            // fire-and-forget; VM has a guard to avoid double prompts
            _ = vm.SavePromptOrAutoSaveAsync();
        }
        catch
        {
            // swallow: don't let UI binding notifications crash the app
        }
    }

    // Global lost-keyboard-focus handler: delegates to VM save/prompt logic but skips caption-controlled TextBoxes.
    private async void Editor_LostKeyboardFocus(object? sender, KeyboardFocusChangedEventArgs e)
    {
        // Only react when the focus left a TextBox (avoid unrelated focus changes)
        if (e.OriginalSource is not TextBox tb) return;

        // If the caption handler already handled this particular TextBox LostFocus, skip global save.
        lock (_captionHandledLock)
        {
            if (_captionHandled.Remove(tb))
            {
                _vm.Log("Skipped global auto-save because caption handler already committed the edit.");
                return;
            }
        }

        // Only handle TextBoxes that belong to the tree (bound to a TreeItemVm)
        if (tb.DataContext is not TreeItemVm) return;
        if (DataContext is not MainWindowViewModel vm) return;

        try
        {
            // Delegate to VM; VM contains a re-entrancy guard so multiple concurrent calls won't prompt twice.
            await vm.SavePromptOrAutoSaveAsync();
        }
        catch (Exception ex)
        {
            _vm.Log($"Save on edit failed: {ex.Message}");
            try { MessageBox.Show(this, $"Failed to save edits: {ex.Message}", "Save error", MessageBoxButton.OK, MessageBoxImage.Error); } catch { }
        }
    }

    // Helper: normalize/clone values for safe comparison (avoid reference-equality surprises)
    private static object? CloneForComparison(object? v)
    {
        if (v == null) return null;
        // For common mutable types used here (string, bool, int, double) Equals is fine.
        // If the VM uses complex objects for Value, you can extend this to produce an immutable representation.
        return v;
    }

    private static bool AreEqualForSave(object? a, object? b)
    {
        if (a is null && b is null) return true;
        if (a is null || b is null) return false;
        // Strings and boxed primitives compare correctly with Equals
        return a.Equals(b);
    }
}
