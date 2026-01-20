using System;
using System.Diagnostics;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Input;

namespace AutoJsonBuilder;

// NOTE: ensure MainWindow.xaml has: xmlns:local="clr-namespace:AutoJsonBuilder"

public partial class MainWindow : Window
{
    private readonly MainWindowViewModel _vm = new();

    public MainWindow()
    {
        InitializeComponent();
        DataContext = _vm;

        _vm.Log("MainWindow ctor reached (post-InitializeComponent).");

        AppDomain.CurrentDomain.FirstChanceException += (_, e) =>
        {
            _vm.Log($"FirstChanceException: {e.Exception.GetType().Name}: {e.Exception.Message}");
        };

        _vm.Log("MainWindow ctor reached.");

        CommandBindings.Add(new CommandBinding(ApplicationCommands.Close, (_, __) => Close()));

        // Trace to a file (works in VS Code + WinExe)
        var logPath = Path.Combine(AppContext.BaseDirectory, "wpf-bindings.log");
        Trace.Listeners.Add(new TextWriterTraceListener(logPath));
        Trace.AutoFlush = true;

        _vm.Log($"Debug started. Trace file: {logPath}");

        // Turn on WPF binding diagnostics (goes to Trace listeners)
        PresentationTraceSources.DataBindingSource.Switch.Level = SourceLevels.Warning | SourceLevels.Error;
        PresentationTraceSources.DataBindingSource.Listeners.Add(new TextWriterTraceListener(logPath));
    }

    private void TreeView_SelectedItemChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
    {
        _vm.SetSelectedTreeItem(e.NewValue as TreeItemVm);
    }
}
