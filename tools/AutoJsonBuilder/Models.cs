namespace AutoJsonBuilder;

public sealed record DumpModel(
    List<FixtureModel> Fixtures,
    List<TargetModel> Targets
);

public sealed record FixtureModel(string Name, double X, double Y);
public sealed record TargetModel(string Name, int ModuleId, double X, double Y);
