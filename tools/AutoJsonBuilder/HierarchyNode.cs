using System.Collections.ObjectModel;

namespace AutoJsonBuilder;

public sealed class HierarchyNode
{
    public string Display { get; }
    public string Details { get; }
    public ObservableCollection<HierarchyNode> Children { get; } = new();

    public HierarchyNode(string display, string details = "")
    {
        Display = display;
        Details = details;
    }
}
