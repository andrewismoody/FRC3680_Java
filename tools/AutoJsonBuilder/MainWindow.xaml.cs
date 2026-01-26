using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Collections.Generic;
using System.Threading.Tasks;
using AutoJsonBuilder.Helpers;

namespace AutoJsonBuilder
{
    public partial class MainWindow : Window
    {
        private readonly MainWindowViewModel _vm = new();

        // helpers to avoid double-handling of caption commits and to detect real value changes
        private readonly HashSet<TextBox> _captionHandled = new();
        private readonly object _captionHandledLock = new();
        private readonly Dictionary<FrameworkElement, object?> _lastBindingValues = new();
        private readonly object _lastBindingValuesLock = new();

        // Layout tuning
        private const double PaneMinContentHeight = 120.0;   // minimum visible content when expander is expanded
        private const double PaneHeaderEstimate = 32.0;      // fallback header estimate (px) used when collapsed
        private const double DefaultPlotMinRatio = 0.5;      // default: plot should occupy at least 50% of right area
        private double _defaultPlotMin = 0.0;                // computed from RightGrid.ActualHeight
        private bool _plotRowLockedToPixel = false;          // set when we temporarily force a pixel height

        // Helper: prefer ScrollViewer viewport height (visible area) if present, otherwise fall back to RightGrid.ActualHeight.
        private double GetRightViewportHeight()
        {
            try
            {
                if (RightScrollViewer != null && RightScrollViewer.ViewportHeight > 0)
                    return RightScrollViewer.ViewportHeight;
            }
            catch { }
            return RightGrid?.ActualHeight ?? 0.0;
        }

        public MainWindow()
        {
            InitializeComponent();
            DataContext = _vm;

            // ensure plot area constraints updated on resize
            this.SizeChanged += MainWindow_SizeChanged;
            // initialize default plot min (use visible viewport if possible)
            try { _defaultPlotMin = Math.Max(0, GetRightViewportHeight() * DefaultPlotMinRatio); PlotRow.MinHeight = _defaultPlotMin; } catch { _defaultPlotMin = 0; }

            // initialize expander rows to header-only if collapsed (designer-safe)
            try
            {
                ExpanderRow1.Height = SelectionExpander.IsExpanded ? new GridLength(1, GridUnitType.Star) : GridLength.Auto;
                ExpanderRow2.Height = DebugExpander.IsExpanded ? new GridLength(1, GridUnitType.Star) : GridLength.Auto;
            }
            catch { }

            // small warm-up to reduce first-visible stalls (best-effort)
            _ = Task.Run(() =>
            {
                try
                {
                    if (Application.Current?.Dispatcher != null)
                    {
                        Application.Current.Dispatcher.Invoke(() =>
                        {
                            try
                            {
                                var dv = new DrawingVisual();
                                var rtb = new RenderTargetBitmap(1, 1, 96, 96, PixelFormats.Pbgra32);
                                rtb.Render(dv);
                            }
                            catch { }
                        });
                    }
                    try { _ = _vm.RebuildTreeAsync(); } catch { }
                }
                catch { }
            });

            CommandBindings.Add(new CommandBinding(ApplicationCommands.Close, (_, __) => Close()));

            // global handlers for focus and binding updates
            AddHandler(Keyboard.LostKeyboardFocusEvent, new KeyboardFocusChangedEventHandler(Editor_LostKeyboardFocus), handledEventsToo: true);
            AddHandler(Keyboard.GotKeyboardFocusEvent, new KeyboardFocusChangedEventHandler(Editor_GotKeyboardFocus), handledEventsToo: true);
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

            lock (_captionHandledLock) { _captionHandled.Add(tb); }
            try
            {
                await _vm.CommitPoseCaptionAsync(item);
            }
            catch { }
            finally
            {
                // remove mark after a short delay in case global handler didn't see it
                _ = Task.Run(async () =>
                {
                    await Task.Delay(1500).ConfigureAwait(false);
                    lock (_captionHandledLock) { _captionHandled.Remove(tb); }
                });
            }
        }

        // When an editor receives focus update selection details to that node
        private void Editor_GotKeyboardFocus(object? sender, KeyboardFocusChangedEventArgs e)
        {
            try
            {
                if (e.NewFocus is FrameworkElement fe && fe.DataContext is TreeItemVm tivm)
                    _vm.SetSelectedTreeItem(tivm);
            }
            catch { }
        }

        // Binding target updated (NotifyOnTargetUpdated = true): detect real value changes and trigger save/autosave
        private void OnBindingTargetUpdated(object? sender, RoutedEventArgs e)
        {
            try
            {
                if (e.OriginalSource is not FrameworkElement fe) return;
                if (fe.DataContext is not TreeItemVm tivm) return;
                if (tivm.Kind != TreeItemKind.Field) return;

                var current = tivm.Value;

                lock (_lastBindingValuesLock)
                {
                    if (!_lastBindingValues.TryGetValue(fe, out var prev))
                    {
                        _lastBindingValues[fe] = CloneForComparison(current);
                        return;
                    }
                    if (AreEqualForSave(prev, current)) return;
                    _lastBindingValues[fe] = CloneForComparison(current);
                }

                _ = _vm.SavePromptOrAutoSaveAsync();
            }
            catch { }
        }

        // Global lost-keyboard-focus handler: delegate to VM save/prompt except when caption handler ran
        private async void Editor_LostKeyboardFocus(object? sender, KeyboardFocusChangedEventArgs e)
        {
            if (e.OriginalSource is not TextBox tb) return;

            lock (_captionHandledLock)
            {
                if (_captionHandled.Remove(tb))
                {
                    _vm.Log("Skipped global auto-save because caption handler already committed the edit.");
                    return;
                }
            }

            if (tb.DataContext is not TreeItemVm) return;

            try
            {
                await _vm.SavePromptOrAutoSaveAsync();
            }
            catch (Exception ex)
            {
                _vm.Log($"Save on edit failed: {ex.Message}");
                try { MessageBox.Show(this, $"Failed to save edits: {ex.Message}", "Save error", MessageBoxButton.OK, MessageBoxImage.Error); } catch { }
            }
        }

        private static object? CloneForComparison(object? v) => v;

        private static bool AreEqualForSave(object? a, object? b)
        {
            if (a is null && b is null) return true;
            if (a is null || b is null) return false;
            return a.Equals(b);
        }

        // Ensure expanders have at least their minimum visible height. If there's not enough room,
        // temporarily reduce the plot row (to a pixel height) to make space. Restore star-sizing
        // when available space is sufficient again.
        private void EnsureSpaceForExpanders()
        {
            try
            {
                // compute required bottom area for expanders (including internal splitter heights)
                double req = 0.0;
                // top expander
                req += SelectionExpander.IsExpanded ? (PaneHeaderEstimate + PaneMinContentHeight) : PaneHeaderEstimate;
                // internal splitter between two expanders
                req += 6.0;
                // bottom expander
                req += DebugExpander.IsExpanded ? (PaneHeaderEstimate + PaneMinContentHeight) : PaneHeaderEstimate;

                // include outer splitter between plot and expanders (height = 6)
                req += 6.0;

                var available = GetRightViewportHeight();

                // desired plot min based on current right-grid size
                _defaultPlotMin = Math.Max(0, GetRightViewportHeight() * DefaultPlotMinRatio);

                // If not enough room for plot's default min + required expanders, reduce plot to make room.
                if (available < _defaultPlotMin + req)
                {
                    // compute new pixel height for plot row that leaves 'req' for bottom area
                    double newPlotPx = Math.Max(24.0, available - req); // never force to 0; keep small visible area
                    PlotRow.Height = new GridLength(newPlotPx, GridUnitType.Pixel);
                    _plotRowLockedToPixel = true;
                    // allow expanders to size to at least their min
                    ExpanderRow1.MinHeight = SelectionExpander.IsExpanded ? (PaneHeaderEstimate + PaneMinContentHeight) : PaneHeaderEstimate;
                    ExpanderRow2.MinHeight = DebugExpander.IsExpanded ? (PaneHeaderEstimate + PaneMinContentHeight) : PaneHeaderEstimate;
                }
                else
                {
                    // there is enough room: restore star sizing and default plot min
                    if (_plotRowLockedToPixel)
                    {
                        PlotRow.Height = new GridLength(3, GridUnitType.Star); // restore proportional sizing
                        _plotRowLockedToPixel = false;
                    }
                    PlotRow.MinHeight = _defaultPlotMin;
                    // set expander min heights so expanded panes show minimum content
                    ExpanderRow1.MinHeight = SelectionExpander.IsExpanded ? (PaneHeaderEstimate + PaneMinContentHeight) : PaneHeaderEstimate;
                    ExpanderRow2.MinHeight = DebugExpander.IsExpanded ? (PaneHeaderEstimate + PaneMinContentHeight) : PaneHeaderEstimate;
                }
            }
            catch
            {
                // best effort
            }
        }

        // Expander handlers: toggle corresponding row between Auto (header-only) and Star (shareable)
        private void Expander_Expanded(object sender, RoutedEventArgs e)
        {
            try
            {
                if (sender == SelectionExpander) ExpanderRow1.Height = new GridLength(1, GridUnitType.Star);
                else if (sender == DebugExpander) ExpanderRow2.Height = new GridLength(1, GridUnitType.Star);
                // ensure enough room to show minimum pane content
                EnsureSpaceForExpanders();
            }
            catch { }
        }

        private void Expander_Collapsed(object sender, RoutedEventArgs e)
        {
            try
            {
                if (sender == SelectionExpander) ExpanderRow1.Height = GridLength.Auto;
                else if (sender == DebugExpander) ExpanderRow2.Height = GridLength.Auto;
                // collapsed -> recompute layout constraints and potentially restore plot sizing
                EnsureSpaceForExpanders();
            }
            catch { }
        }

        private void MainWindow_SizeChanged(object? sender, SizeChangedEventArgs e)
        {
            try
            {
                // recompute default plot min and ensure expanders can get their minimum if needed
                _defaultPlotMin = Math.Max(0, GetRightViewportHeight() * DefaultPlotMinRatio);
                // only enforce as MinHeight (can be temporarily overridden when expanders need space)
                if (!_plotRowLockedToPixel) PlotRow.MinHeight = _defaultPlotMin;
                EnsureSpaceForExpanders();
            }
            catch
            {
                // best-effort only
            }
        }
    }
}
