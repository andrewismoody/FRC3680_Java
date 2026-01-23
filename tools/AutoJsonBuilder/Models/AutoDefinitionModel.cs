using System.Collections.Generic;

namespace AutoJsonBuilder.Models;

public sealed class AutoDefinitionModel
{
    public string Season { get; set; } = "2025";
    public string Version { get; set; } = "1";
    public string Description { get; set; } = "";

    public Dictionary<string, ParamValue> Params { get; set; } = new();

    public List<PoseModel> Poses { get; set; } = new();
    public List<string> Groups { get; set; } = new();
    public List<string> Modules { get; set; } = new();
    public List<string> TravelGroups { get; set; } = new();
    public List<string> Locations { get; set; } = new();
    public List<string> Positions { get; set; } = new();
    public List<string> Actions { get; set; } = new();

    public List<EventModel> Events { get; set; } = new();
    public List<TargetSchemaModel> Targets { get; set; } = new();
    public List<FixtureSchemaModel> Fixtures { get; set; } = new();
    public List<SequenceModel> Sequences { get; set; } = new();
}

public sealed class PoseModel
{
    public string? Name { get; set; }
    public string Group { get; set; } = "";
    public string Location { get; set; } = "";
    public int Index { get; set; }
    public string Position { get; set; } = "";
    public string Action { get; set; } = "";
}

public sealed class SequenceModel
{
    public string Name { get; set; } = "";
    public List<string>? Start1 { get; set; }
    public List<string>? Start2 { get; set; }
    public List<string>? Start3 { get; set; }
    public List<string> Events { get; set; } = new();
}

public sealed class EventModel
{
    public string Name { get; set; } = "";
    public string Type { get; set; } = "await"; // await|time
    public bool Parallel { get; set; }

    public string? Pose { get; set; }
    public int? Milliseconds { get; set; }

    public string? TriggerType { get; set; } // boolean|none
    public bool? TriggerValue { get; set; }
    public string? TriggerModule { get; set; }
    public bool TriggerInvert { get; set; }
}

public sealed class TargetSchemaModel
{
    public string Module { get; set; } = "";
    public string? Group { get; set; }
    public string? Location { get; set; }
    public int? Index { get; set; }
    public string? Position { get; set; }
    public string? Action { get; set; }

    public double? Measurement { get; set; }
    public string? State { get; set; } // Off|Forward|Reverse

    public TranslationModel? Translation { get; set; }
    public FixtureRefModel? Fixture { get; set; }
}

public sealed class FixtureSchemaModel
{
    public string Type { get; set; } = "";
    public int Index { get; set; }
    public TranslationModel? Translation { get; set; }
    public DerivedFromModel? DerivedFrom { get; set; }
}

public sealed class FixtureRefModel
{
    public string Type { get; set; } = "";
    public int Index { get; set; }
}

public sealed class TranslationModel
{
    public List<object>? Position { get; set; } // number|string per schema
    public string? PositionUnits { get; set; } // meters|inches
    public object? Rotation { get; set; } // number|string
    public string? RotationUnits { get; set; } // radians|degrees
}

public sealed class DerivedFromModel
{
    public FixtureRefModel? Fixture { get; set; }
    public FixtureRefModel? Fixture2 { get; set; }
    public DerivedFromModel? DerivedFrom { get; set; }
    public DerivedFromModel? DerivedFrom2 { get; set; }

    public string Function { get; set; } = "none"; // parallel|perpendicular|bisector|none
    public object? Offset { get; set; } // number|string (param)
}
