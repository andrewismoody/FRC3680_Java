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
using OxyPlot;            // <-- added: provide PlotModel, OxyRect, etc.
using OxyPlot.Axes;      // <-- added: provides AxisPosition, Axis types used in diagnostics
using OxyPlot.Wpf; // ensure OxyPlot.Wpf is available in the UI project

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

        // throttle logging so mouse-move diagnostics aren't too chatty
        private DateTime _lastMouseDiag = DateTime.MinValue;
        private readonly TimeSpan _mouseDiagInterval = TimeSpan.FromMilliseconds(250);

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

            // wire plot clicks (attempts to find the PlotView control by common names or by visual-tree search)
            HookPlotViewMouse();

            // Try again after Loaded when visual tree is guaranteed; useful if HookPlotViewMouse ran too early.
            this.Loaded -= MainWindow_Loaded;
            this.Loaded += MainWindow_Loaded;
        }

        private void MainWindow_Loaded(object? sender, RoutedEventArgs e)
        {
            try
            {
                _vm.Log("MainWindow.Loaded: calling HookPlotViewMouse (second attempt)");
                HookPlotViewMouse();
            }
            catch (Exception ex) { _vm.Log($"MainWindow_Loaded: HookPlotViewMouse threw: {ex.Message}"); }
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

        // Try to locate the PlotView control and attach MouseDown to forward clicks to the VM.
        private void HookPlotViewMouse()
        {
            try
            {
                _vm.Log("HookPlotViewMouse: start");
                PlotView pv = null;
                // try common names used in XAML
                var candidateNames = new[] { "FieldPlotView", "fieldPlotView", "PlotView", "plotView", "plotView1" };
                foreach (var n in candidateNames)
                {
                    _vm.Log($"HookPlotViewMouse: checking FindName('{n}')");
                    var found = this.FindName(n) as PlotView;
                    if (found != null) { pv = found; break; }
                }

                // fallback: search visual tree for first PlotView
                if (pv == null) pv = FindVisualChild<PlotView>(this);

                if (pv != null)
                {
                    _vm.Log($"HookPlotViewMouse: found PlotView (Name='{pv.Name ?? "(unnamed)"}').");
                    try { if (pv.Background == null) _vm.Log("HookPlotViewMouse: PlotView.Background is null — consider setting Background=\"Transparent\" to receive clicks."); } catch { }
                }
                else
                {
                    _vm.Log("HookPlotViewMouse: no PlotView found (FindName + visual-tree search failed).");
                }

                if (pv != null)
                {
                    // ensure any previously attached handlers are removed, but do NOT attach new ones.
                    // This disables all PlotView mouse-based legend/cursor behavior.
                    try
                    {
                        pv.MouseDown -= PlotView_MouseDown;
                        pv.MouseMove -= PlotView_MouseMove;
                        pv.MouseLeave -= PlotView_MouseLeave;
                    }
                    catch { /* best-effort */ }

                    _vm.Log("HookPlotViewMouse: mouse handlers detached; PlotView mouse handling disabled.");
                }
            }
            catch (Exception ex)
            {
                _vm.Log($"HookPlotViewMouse: exception: {ex.Message}");
                /* best-effort */
            }
        }

        // MouseDown handler: get mouse pos relative to PlotView and forward to VM helper.
        private void PlotView_MouseDown(object? sender, MouseButtonEventArgs e)
        {
            try
            {
                var pv = sender as PlotView ?? FindVisualChild<PlotView>(this);
                if (pv == null) return;
                if (DataContext is not MainWindowViewModel vm) return;

                var pt = e.GetPosition(pv); // PlotView-relative (DIP)
                // IMPORTANT: do NOT DPI-scale here — PlotHelper expects PlotView-relative DIPs.
                try { _vm.Log($"PlotView.MouseDown: raw={pt.X:F1},{pt.Y:F1} (PlotView-relative DIP)"); } catch { }
                var handled = vm.HandlePlotMouseDown(pt.X, pt.Y);

                try { _vm.Log($"PlotView.MouseDown: handled={handled}"); } catch { }
                if (handled)
                {
                    try { pv.Model?.InvalidatePlot(false); } catch { }
                    e.Handled = true;
                }
            }
            catch { /* best-effort */ }
        }

        // MouseMove: show cursor indicator at PlotView-relative point
        private void PlotView_MouseMove(object? sender, MouseEventArgs e)
        {
            try
            {
                var pv = sender as PlotView ?? FindVisualChild<PlotView>(this);
                if (pv == null) return;
                if (DataContext is not MainWindowViewModel vm) return;

                var pt = e.GetPosition(pv); // PlotView-relative (DIP)
                // IMPORTANT: send the raw PlotView-relative point (DIP) to the VM/PlotHelper.
                // If needed, log screen coords separately for diagnostics:
                var screenPt = pv.PointToScreen(pt);

                // Throttled diagnostics: compare coordinate spaces and PlotModel plotArea / axes mappings
                var now = DateTime.UtcNow;
                if (now - _lastMouseDiag > _mouseDiagInterval)
                {
                    _lastMouseDiag = now;
                    try
                    {
                        var pm = pv.Model;
                        // WPF screen point (device-independent) -> point in screen (device) coords
                        vm.Log($"MouseDiag: pv.pt(DIP)={pt.X:F1},{pt.Y:F1} pv.PointToScreen={screenPt.X:F1},{screenPt.Y:F1}");

                        if (pm != null)
                        {
                            var plotArea = pm.PlotArea;
                            vm.Log($"MouseDiag: PlotModel.PlotArea LeftTop={plotArea.Left:F1},{plotArea.Top:F1} Size={plotArea.Width:F1}x{plotArea.Height:F1}");
                            var xAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Bottom) ?? pm.Axes.FirstOrDefault();
                            var yAxis = pm.Axes.FirstOrDefault(a => a.Position == AxisPosition.Left) ?? pm.Axes.Skip(1).FirstOrDefault() ?? xAxis;
                            if (xAxis != null && yAxis != null && plotArea.Width > 0 && plotArea.Height > 0)
                            {
                                // log PlotView origin and plotArea top-left in screen coordinates to locate absolute offset
                                try
                                {
                                    var pvOriginScreen = pv.PointToScreen(new Point(0, 0));
                                    var plotAreaTopLeftScreen = pv.PointToScreen(new Point(plotArea.Left, plotArea.Top));
                                    vm.Log($"MouseDiag: pv.origin.screen={pvOriginScreen.X:F1},{pvOriginScreen.Y:F1} plotArea.topLeft.screen={plotAreaTopLeftScreen.X:F1},{plotAreaTopLeftScreen.Y:F1}");
                                }
                                catch { /* best-effort */ }

                                // what data point do we compute from current PlotView-relative DIPs (what ShowCursorIndicator uses)
                                var localX = pt.X - plotArea.Left;
                                var localY = pt.Y - plotArea.Top;
                                double dataX = 0, dataY = 0;
                                try { dataX = xAxis.InverseTransform(localX); } catch { }
                                try { dataY = yAxis.InverseTransform(localY); } catch { }
                                vm.Log($"MouseDiag: mapped local={localX:F1},{localY:F1} -> data={dataX:F4},{dataY:F4}");

                                // Re-project that data point back to PlotView surface using axis.Transform and plotArea offset,
                                // then map to screen coords so we can compare against pv.PointToScreen(pt).
                                try
                                {
                                    var backX = plotArea.Left + xAxis.Transform(dataX);    // screen px relative to PlotView
                                    var backY = plotArea.Top  + yAxis.Transform(dataY);    // screen px relative to PlotView
                                    var backScreen = pv.PointToScreen(new Point(backX, backY));
                                    var mouseScreen = pv.PointToScreen(pt);
                                    var dx = mouseScreen.X - backScreen.X;
                                    var dy = mouseScreen.Y - backScreen.Y;
                                    vm.Log($"MouseDiag: reproj.screen={backScreen.X:F1},{backScreen.Y:F1} mouse.screen={mouseScreen.X:F1},{mouseScreen.Y:F1} delta={dx:F1},{dy:F1}");
                                }
                                catch { /* best-effort */ }
 
                                // report first few legend screen rects if available
                                if (PlotHelper_GetLegendRegionsCount(pm) > 0)
                                {
                                    var regs = PlotHelper_GetLegendRegionsSnapshot(pm);
                                    for (int i = 0; i < Math.Min(3, regs.Count); i++)
                                    {
                                        var r = regs[i];
                                        vm.Log($"MouseDiag: legend[{i}] rect={r.Left:F1},{r.Top:F1} {r.Width:F1}x{r.Height:F1}");
                                    }
                                }
                            }
                        }
                    }
                    catch (Exception ex) { try { vm.Log($"MouseDiag: exception: {ex.Message}"); } catch { } }
                }

                // forward raw PlotView-relative coords to VM/PlotHelper
                if (vm.HandlePlotMouseMove(pt.X, pt.Y))
                {
                    try { pv.Model?.InvalidatePlot(false); } catch { }
                }
            }
            catch { /* best-effort */ }
        }

        private void PlotView_MouseLeave(object? sender, MouseEventArgs e)
        {
            try
            {
                var pv = sender as PlotView ?? FindVisualChild<PlotView>(this);
                if (pv == null) return;
                if (DataContext is not MainWindowViewModel vm) return;
                vm.HandlePlotMouseLeave();
                try { pv.Model?.InvalidatePlot(false); } catch { }
            }
            catch { /* best-effort */ }
        }

        // utility: search visual tree for first child of type T
        private static T? FindVisualChild<T>(DependencyObject? parent) where T : DependencyObject
        {
            if (parent == null) return null;
            var count = VisualTreeHelper.GetChildrenCount(parent);
            for (int i = 0; i < count; i++)
            {
                var child = VisualTreeHelper.GetChild(parent, i);
                if (child is T t) return t;
                var result = FindVisualChild<T>(child);
                if (result != null) return result;
            }
            return null;
        }

        // Small accessor helpers that read legend region ScreenRect info from PlotHelper (safe accessor wrapper).
        // These call into the helper via reflection to avoid exposing internal collections publicly.
        private static List<OxyPlot.OxyRect> PlotHelper_GetLegendRegionsSnapshot(PlotModel pm)
        {
            var outList = new List<OxyPlot.OxyRect>();
            try
            {
                var t = typeof(AutoJsonBuilder.Helpers.PlotHelper);
                var field = t.GetField("s_legendRegions", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Static);
                if (field == null) return outList;
                var dict = field.GetValue(null) as System.Collections.IDictionary;
                if (dict == null) return outList;
                if (!dict.Contains(pm)) return outList;
                var list = dict[pm] as System.Collections.IEnumerable;
                if (list == null) return outList;
                foreach (var e in list)
                {
                    var prop = e.GetType().GetProperty("ScreenRect");
                    if (prop != null)
                    {
                        var val = prop.GetValue(e);
                        if (val is OxyPlot.OxyRect r) outList.Add(r);
                    }
                }
            }
            catch { }
            return outList;
        }

        private static int PlotHelper_GetLegendRegionsCount(PlotModel pm)
        {
            try
            {
                var t = typeof(AutoJsonBuilder.Helpers.PlotHelper);
                var field = t.GetField("s_legendRegions", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Static);
                if (field == null) return 0;
                var dict = field.GetValue(null) as System.Collections.IDictionary;
                if (dict == null) return 0;
                if (!dict.Contains(pm)) return 0;
                var list = dict[pm] as System.Collections.ICollection;
                return list?.Count ?? 0;
            }
            catch { return 0; }
        }
    }
}
