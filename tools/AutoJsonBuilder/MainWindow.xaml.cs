using System.Windows;
using System.Windows.Controls;
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

        CommandBindings.Add(new CommandBinding(ApplicationCommands.Close, (_, __) => Close()));
    }

    private void TreeView_SelectedItemChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
    {
        _vm.SetSelectedTreeItem(e.NewValue as TreeItemVm);
    }
}
