using System.Collections.Generic;
using System.Text.Json.Serialization;

namespace AutoJsonBuilder.Models;

public sealed class AutoDefinitionModel
{
    [JsonPropertyName("season")]
    public string Season { get; set; } = "2025";

    [JsonPropertyName("version")]
    public string Version { get; set; } = "1";

    [JsonPropertyName("description")]
    public string Description { get; set; } = "";

    [JsonPropertyName("params")]
    public Dictionary<string, ParamValue> Params { get; set; } = new();

    [JsonPropertyName("poses")]
    public List<PoseModel> Poses { get; set; } = new();

    [JsonPropertyName("groups")]
    public List<string> Groups { get; set; } = new();

    [JsonPropertyName("modules")]
    public List<string> Modules { get; set; } = new();

    [JsonPropertyName("travelGroups")]
    public List<string> TravelGroups { get; set; } = new();

    [JsonPropertyName("locations")]
    public List<string> Locations { get; set; } = new();

    [JsonPropertyName("positions")]
    public List<string> Positions { get; set; } = new();

    [JsonPropertyName("actions")]
    public List<string> Actions { get; set; } = new();

    [JsonPropertyName("events")]
    public List<EventModel> Events { get; set; } = new();

    [JsonPropertyName("targets")]
    public List<TargetSchemaModel> Targets { get; set; } = new();

    [JsonPropertyName("fixtures")]
    public List<FixtureSchemaModel> Fixtures { get; set; } = new();

    [JsonPropertyName("sequences")]
    public List<SequenceModel> Sequences { get; set; } = new();
}

public sealed class PoseModel
{
    [JsonPropertyName("name")]
    public string? Name { get; set; }

    [JsonPropertyName("group")]
    public string Group { get; set; } = "";

    [JsonPropertyName("location")]
    public string Location { get; set; } = "";

    [JsonPropertyName("index")]
    public int Index { get; set; }

    [JsonPropertyName("position")]
    public string Position { get; set; } = "";

    [JsonPropertyName("action")]
    public string Action { get; set; } = "";
}

public sealed class SequenceModel
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";

    [JsonPropertyName("start1")]
    public List<string>? Start1 { get; set; }

    [JsonPropertyName("start2")]
    public List<string>? Start2 { get; set; }

    [JsonPropertyName("start3")]
    public List<string>? Start3 { get; set; }

    [JsonPropertyName("events")]
    public List<string> Events { get; set; } = new();
}

public sealed class EventModel
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";

    [JsonPropertyName("type")]
    public string Type { get; set; } = "await";

    [JsonPropertyName("parallel")]
    public bool Parallel { get; set; }

    [JsonPropertyName("pose")]
    public string? Pose { get; set; }

    [JsonPropertyName("milliseconds")]
    public int? Milliseconds { get; set; }

    [JsonPropertyName("triggerType")]
    public string? TriggerType { get; set; }

    [JsonPropertyName("triggerValue")]
    public bool? TriggerValue { get; set; }

    [JsonPropertyName("triggerModule")]
    public string? TriggerModule { get; set; }

    [JsonPropertyName("triggerInvert")]
    public bool TriggerInvert { get; set; }
}

public sealed class TargetSchemaModel
{
    [JsonPropertyName("module")]
    public string Module { get; set; } = "";

    [JsonPropertyName("group")]
    public string? Group { get; set; }

    [JsonPropertyName("location")]
    public string? Location { get; set; }

    [JsonPropertyName("index")]
    public int? Index { get; set; }

    [JsonPropertyName("position")]
    public string? Position { get; set; }

    [JsonPropertyName("action")]
    public string? Action { get; set; }

    [JsonPropertyName("measurement")]
    public double? Measurement { get; set; }

    [JsonPropertyName("state")]
    public string? State { get; set; }

    [JsonPropertyName("translation")]
    public TranslationModel? Translation { get; set; }

    [JsonPropertyName("fixture")]
    public FixtureRefModel? Fixture { get; set; }

    [JsonPropertyName("lookAtTranslation")]
    public TranslationModel? LookAtTranslation { get; set; }

    [JsonPropertyName("lookAtFixture")]
    public FixtureRefModel? LookAtFixture { get; set; }
}

public sealed class FixtureSchemaModel
{
    [JsonPropertyName("type")]
    public string Type { get; set; } = "";

    [JsonPropertyName("index")]
    public int Index { get; set; }

    [JsonPropertyName("translation")]
    public TranslationModel? Translation { get; set; }

    [JsonPropertyName("derivedFrom")]
    public DerivedFromModel? DerivedFrom { get; set; }
}

public sealed class FixtureRefModel
{
    [JsonPropertyName("type")]
    public string Type { get; set; } = "";

    [JsonPropertyName("index")]
    public int Index { get; set; }
}

public sealed class TranslationModel
{
    [JsonPropertyName("position")]
    public List<object>? Position { get; set; }

    [JsonPropertyName("positionUnits")]
    public string? PositionUnits { get; set; }

    [JsonPropertyName("rotation")]
    public object? Rotation { get; set; }

    [JsonPropertyName("rotationUnits")]
    public string? RotationUnits { get; set; }
}

public sealed class DerivedFromModel
{
    [JsonPropertyName("fixture")]
    public FixtureRefModel? Fixture { get; set; }

    [JsonPropertyName("fixture2")]
    public FixtureRefModel? Fixture2 { get; set; }

    [JsonPropertyName("derivedFrom")]
    public DerivedFromModel? DerivedFrom { get; set; }

    [JsonPropertyName("derivedFrom2")]
    public DerivedFromModel? DerivedFrom2 { get; set; }

    [JsonPropertyName("function")]
    public string Function { get; set; } = "none";

    [JsonPropertyName("offset")]
    public object? Offset { get; set; }
}
