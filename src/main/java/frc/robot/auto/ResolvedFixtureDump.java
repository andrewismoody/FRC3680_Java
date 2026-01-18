package frc.robot.auto;

import java.util.Map;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 * CLI tool:
 *   java ... frc.robot.auto.ResolvedFixtureDump --file src/main/deploy/auto2025.json
 *
 * Outputs JSON to stdout:
 * {
 *   "fixtures": {
 *     "type:index": { "x":..., "y":..., "z":..., "units":"meters" }
 *   }
 * }
 */
public final class ResolvedFixtureDump {
    private ResolvedFixtureDump() {}

    public static void main(String[] args) throws Exception {
        String file = "src/main/deploy/auto2025.json";
        boolean debug = false;

        for (int i = 0; i < args.length; i++) {
            if ("--file".equals(args[i]) && i + 1 < args.length) {
                file = args[++i];
            } else if ("--debug".equals(args[i])) {
                debug = true;
            }
        }

        if (debug) System.err.println("[ResolvedFixtureDump] file=" + file);

        ObjectMapper om = new ObjectMapper();

        AutoParser.ParsedDefinitions defs = AutoParser.LoadDefinitions(file);
        AutoSeasonDefinition def = defs.def;

        if (debug) System.err.println("[ResolvedFixtureDump] resolving fixtures...");
        FixtureResolver.ResolveAll(def, false, debug);
        if (debug) System.err.println("[ResolvedFixtureDump] resolvedFixtures=" + def.resolvedFixtures.size());

        ObjectNode out = om.createObjectNode();
        ObjectNode fixturesOut = om.createObjectNode();
        out.set("fixtures", fixturesOut);

        for (Map.Entry<String, JsonNode> e : def.resolvedFixtures.entrySet()) {
            JsonNode fixture = e.getValue();
            if (fixture == null || !fixture.isObject()) continue;

            JsonNode t = fixture.get("translation");
            if (t == null || !t.isObject()) continue;

            JsonNode pos = t.get("position");
            if (pos == null || !pos.isArray() || pos.size() < 2) continue;

            double x = pos.get(0).asDouble(0.0);
            double y = pos.get(1).asDouble(0.0);
            double z = (pos.size() > 2) ? pos.get(2).asDouble(0.0) : 0.0;

            ObjectNode one = om.createObjectNode();
            one.put("x", x);
            one.put("y", y);
            one.put("z", z);
            one.put("units", t.path("positionUnits").asText("meters"));

            fixturesOut.set(e.getKey(), one);
        }

        System.out.println(om.writeValueAsString(out));
    }
}
