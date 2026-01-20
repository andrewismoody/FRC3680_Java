using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;

namespace AutoJsonBuilder.Behaviors;

public static class TreeViewSelectedItemBehavior
{
    public static readonly DependencyProperty AutoFocusSelectedItemProperty =
        DependencyProperty.RegisterAttached(
            "AutoFocusSelectedItem",
            typeof(bool),
            typeof(TreeViewSelectedItemBehavior),
            new PropertyMetadata(false, OnChanged));

    public static void SetAutoFocusSelectedItem(DependencyObject element, bool value) =>
        element.SetValue(AutoFocusSelectedItemProperty, value);

    public static bool GetAutoFocusSelectedItem(DependencyObject element) =>
        (bool)element.GetValue(AutoFocusSelectedItemProperty);

    private static void OnChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
    {
        if (d is not TreeView tv) return;

        if ((bool)e.NewValue)
            tv.SelectedItemChanged += Tv_SelectedItemChanged;
        else
            tv.SelectedItemChanged -= Tv_SelectedItemChanged;
    }

    private static void Tv_SelectedItemChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
    {
        if (sender is not TreeView tv) return;

        tv.Dispatcher.BeginInvoke(DispatcherPriority.Background, () =>
        {
            if (tv.ItemContainerGenerator.ContainerFromItem(e.NewValue) is TreeViewItem tvi)
            {
                tvi.BringIntoView();
                tvi.Focus();
            }
        });
    }
}
