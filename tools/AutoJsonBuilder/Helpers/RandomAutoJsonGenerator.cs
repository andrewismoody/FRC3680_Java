using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using AutoJsonBuilder.Models;

namespace AutoJsonBuilder.Helpers
{
    // Generates randomized but schema-valid auto.json documents that exercise parsing/factory paths.
    public static class RandomAutoJsonGenerator
    {
        // Generate and optionally write to disk. Use seed for reproducible cases.
        public static string Generate(int seed = 0, bool writeToFile = false, string? outPath = null)
        {
            var rnd = new Random(seed);
            var doc = new AutoDefinitionModel
            {
                Season = $"SE{rnd.Next(2000,9999)}",
                Version = "1",
                Description = $"Random auto doc seed={seed}"
            };

            // Ensure backing lists exist
            doc.Modules ??= new List<string>();
            doc.Groups ??= new List<string>();
            doc.Locations ??= new List<string>();
            doc.Positions ??= new List<string>();
            doc.Actions ??= new List<string>();
            doc.Poses ??= new List<PoseModel>();
            doc.Events ??= new List<EventModel>();
            doc.Fixtures ??= new List<FixtureSchemaModel>();
            doc.Targets ??= new List<TargetSchemaModel>();
            doc.Sequences ??= new List<SequenceModel>();
            doc.Params ??= new Dictionary<string, ParamValue>(StringComparer.OrdinalIgnoreCase);

            // Modules / labels
            doc.Modules.AddRange(new[] { "drive", "arm", "intake" }.Take(rnd.Next(1,4)));
            doc.Groups.AddRange(new[] { "g1", "g2" }.Take(Math.Max(1, rnd.Next(1,3))));
            doc.Locations.AddRange(new[] { "locA", "locB" }.Take(Math.Max(1, rnd.Next(1,3))));
            doc.Positions.AddRange(new[] { "pos1", "pos2" }.Take(Math.Max(1, rnd.Next(1,3))));
            doc.Actions.AddRange(new[] { "act1", "act2" }.Take(Math.Max(1, rnd.Next(1,3))));

            // provided params (use ParamValue.Raw to mirror model expectations)
            void P(string k, object v) => doc.Params.TryAdd(k, new ParamValue { Raw = v });
            P("fieldSize", new List<object> { 690, 317 }); // provided default
            P("fieldCenter", new List<object> { "$fieldSize.X / 2", "$fieldSize.Y / 2" });
            P("frameSize", new List<object> { 24, 24 });
            P("frameCenter", new List<object> { "$frameSize.X / 2", "$frameSize.Y / 2" });
            P("bumperWidth", 0);
            P("startArea", new List<object> { 90, 30 });
            P("redStartTranslation", new List<object> { "$fieldCenter.X", "$fieldCenter.Y", 0 });
            P("redStartRotation", new List<object> { 0, 0, 180 });

            // add some random params (scalars, arrays, expressions)
            for (int i = 0; i < 3; i++)
                P($"extraParam{i}", rnd.NextDouble() * 100);
            P("exprParam", "$fieldSize.X / 3");

            // Poses: create multiple with names and use different Position labels
            for (int i = 1; i <= 3; i++)
            {
                var name = $"pose{i}";
                doc.Poses.Add(new PoseModel
                {
                    Name = name,
                    Group = doc.Groups.FirstOrDefault() ?? "g1",
                    Location = doc.Locations.FirstOrDefault() ?? "locA",
                    Index = i - 1,
                    Position = doc.Positions.FirstOrDefault() ?? "pos1",
                    Action = doc.Actions.FirstOrDefault() ?? "act1"
                });
            }

            // Fixtures: create some with explicit translations, some with only translation that will exercise resolved fixture lookup
            for (int i = 0; i < 4; i++)
            {
                var t = new FixtureSchemaModel
                {
                    Type = $"fixtureType{(i % 2)}",
                    Index = i,
                    Translation = new TranslationModel
                    {
                        Position = new List<object> { i * 10.0, i * 5.0, 0.0 },
                        PositionUnits = (i % 2 == 0) ? "inches" : "m",
                        Rotation = (i % 3 == 0) ? 90.0 : (double?)null,
                        RotationUnits = (i % 3 == 0) ? "degrees" : null
                    }
                };
                doc.Fixtures.Add(t);
            }

            // Targets: include measurement, state, translation, fixture ref, lookAtTranslation, lookAtFixture variants
            // measurement
            doc.Targets.Add(new TargetSchemaModel { Measurement = 12.34 });

            // state (string will be serialized; parser tries to map to enum)
            doc.Targets.Add(new TargetSchemaModel { State = "IDLE", Module = doc.Modules.FirstOrDefault() ?? "drive" });

            // translation with rotation (degrees) and positionUnits inches
            doc.Targets.Add(new TargetSchemaModel
            {
                Module = doc.Modules.FirstOrDefault() ?? "drive",
                Translation = new TranslationModel
                {
                    Position = new List<object> { 120.0, 60.0, 0.0 },
                    PositionUnits = "inches",
                    Rotation = 180.0,
                    RotationUnits = "degrees"
                }
            });

            // translation with expression positional values (string param keys)
            doc.Targets.Add(new TargetSchemaModel
            {
                Module = doc.Modules.Skip(1).FirstOrDefault() ?? "arm",
                Translation = new TranslationModel
                {
                    Position = new List<object> { "$fieldCenter.X", "$fieldCenter.Y", 0 },
                    PositionUnits = "m"
                }
            });

            // fixture reference target
            doc.Targets.Add(new TargetSchemaModel
            {
                Module = doc.Modules.LastOrDefault() ?? "intake",
                Fixture = new FixtureRefModel { Type = "fixtureType0", Index = 0 }
            });

            // target with lookAtTranslation (position-only) to exercise lookAt branches
            doc.Targets.Add(new TargetSchemaModel
            {
                Module = doc.Modules.FirstOrDefault() ?? "drive",
                Translation = new TranslationModel
                {
                    Position = new List<object> { 10.0, 20.0, 0.0 },
                    PositionUnits = "inches"
                },
                LookAtTranslation = new TranslationModel
                {
                    Position = new List<object> { 0.0, 0.0, 0.0 },
                    PositionUnits = "inches"
                }
            });

            // Events: await events referencing poses + time events with trigger (boolean)
            doc.Events.Add(new EventModel { Name = "evtAwait1", Type = "await", Parallel = false, Pose = doc.Poses[0].Name });
            doc.Events.Add(new EventModel { Name = "evtAwait2", Type = "await", Parallel = true, Pose = doc.Poses[1].Name });

            var timeEvt = new EventModel { Name = "evtTime1", Type = "time", Parallel = false, Milliseconds = 500, TriggerType = "boolean", TriggerModule = doc.Modules.FirstOrDefault() ?? "drive", TriggerValue = true, TriggerInvert = false };
            doc.Events.Add(timeEvt);

            // Sequences: include start1/2/3 and events list
            doc.Sequences.Add(new SequenceModel
            {
                Name = "seqAll",
                Start1 = new List<string> { "evtAwait1" },
                Start2 = new List<string> { "evtAwait2" },
                Events = new List<string> { "evtAwait1", "evtTime1", "evtAwait2" }
            });

            // Add a sequence with empty start arrays to exercise other branch
            doc.Sequences.Add(new SequenceModel
            {
                Name = "seqSimple",
                Events = new List<string> { "evtTime1" }
            });

            // Some benign additional coverage: include lookAtFixture references by pointing to fixture 1
            doc.Targets.Add(new TargetSchemaModel
            {
                Module = doc.Modules.First(),
                Translation = new TranslationModel
                {
                    Position = new List<object> { 30.0, 40.0, 0.0 },
                    PositionUnits = "inches"
                },
                LookAtFixture = new FixtureRefModel { Type = "fixtureType1", Index = 1 }
            });

            // Finally: ensure canonical provided param names are present (ParamsInitializer also does this, but include here for completeness)
            // (already added some above; this is a no-op safe step)
            // Serialize with project's camel-case serializer if available
            string json;
            try
            {
                // prefer project's helper if present
                var helperType = typeof(JsonHelper);
                json = JsonHelper.SerializeCamel(doc);
            }
            catch
            {
                // fallback
                var opts = new JsonSerializerOptions { PropertyNamingPolicy = JsonNamingPolicy.CamelCase, WriteIndented = true };
                json = JsonSerializer.Serialize(doc, opts);
            }

            if (writeToFile)
            {
                var path = outPath ?? Path.Combine(AppContext.BaseDirectory ?? ".", $"random-auto-seed-{seed}.json");
                File.WriteAllText(path, json);
            }

            return json;
        }
    }
}
