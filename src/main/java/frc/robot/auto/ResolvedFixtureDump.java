package frc.robot.auto;

import java.util.Map;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.misc.Utility;

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
        boolean allianceTransform = false;

        Double fieldSizeXIn = null;
        Double fieldSizeYIn = null;

        Double redStartXIn = 0.0;
        Double redStartYIn = 0.0;
        Double redStartYawDeg = 0.0;

        Alliance alliance = null;
        Integer driverLocation = null;

        for (int i = 0; i < args.length; i++) {
            String a = args[i];

            if ("--file".equals(a) && i + 1 < args.length) {
                file = args[++i];
            } else if ("--debug".equals(a)) {
                debug = true;
            } else if ("--alliance-transform".equals(a)) {
                allianceTransform = true;
            } else if ("--field-size-x".equals(a) && i + 1 < args.length) {          // CHANGED
                fieldSizeXIn = Double.parseDouble(args[++i]);
            } else if ("--field-size-y".equals(a) && i + 1 < args.length) {          // CHANGED
                fieldSizeYIn = Double.parseDouble(args[++i]);
            } else if ("--red-start-x".equals(a) && i + 1 < args.length) {           // CHANGED
                redStartXIn = Double.parseDouble(args[++i]);
            } else if ("--red-start-y".equals(a) && i + 1 < args.length) {           // CHANGED
                redStartYIn = Double.parseDouble(args[++i]);
            } else if ("--red-start-yaw-deg".equals(a) && i + 1 < args.length) {
                redStartYawDeg = Double.parseDouble(args[++i]);
            } else if ("--alliance".equals(a) && i + 1 < args.length) {
                String v = args[++i].trim().toLowerCase();
                if ("red".equals(v)) alliance = Alliance.Red;
                else if ("blue".equals(v)) alliance = Alliance.Blue;
            } else if ("--driver-location".equals(a) && i + 1 < args.length) {
                driverLocation = Integer.parseInt(args[++i]);
            }
        }

        if (debug) System.err.println("[ResolvedFixtureDump] file=" + file);
        if (debug) System.err.println("[ResolvedFixtureDump] allianceTransform=" + allianceTransform);

        if (allianceTransform) {
            if (fieldSizeXIn == null || fieldSizeYIn == null) {
                throw new IllegalArgumentException("--alliance-transform requires --field-size-x and --field-size-y (inches)");
            }

            double fieldXM = Utility.inchesToMeters(fieldSizeXIn.doubleValue());
            double fieldYM = Utility.inchesToMeters(fieldSizeYIn.doubleValue());
            Utility.setFieldSize(new Translation2d(fieldXM, fieldYM));

            double redXM = Utility.inchesToMeters(redStartXIn);
            double redYM = Utility.inchesToMeters(redStartYIn);

            Transform3d redStart = new Transform3d(
                    new Translation3d(redXM, redYM, 0.0),
                    new Rotation3d(0.0, 0.0, Utility.degreesToRadians(redStartYawDeg)));
            Utility.setRedStartTransform(redStart);

            if (debug) {
                System.err.println("[ResolvedFixtureDump] fieldSize(in)=(" + fieldSizeXIn + "," + fieldSizeYIn + ")");
                System.err.println("[ResolvedFixtureDump] fieldSize(m)=(" + fieldXM + "," + fieldYM + ")");
                System.err.println("[ResolvedFixtureDump] redStart(in)=(" + redStartXIn + "," + redStartYIn + ") yawDeg=" + redStartYawDeg);
                System.err.println("[ResolvedFixtureDump] redStart(m)=(" + redXM + "," + redYM + ", 0.0)");
            }
        }

        if (alliance != null) Utility.setAllianceOverride(alliance);
        if (driverLocation != null) Utility.setDriverLocationOverride(driverLocation);

        if (debug && alliance != null) System.err.println("[ResolvedFixtureDump] allianceOverride=" + alliance);
        if (debug && driverLocation != null) System.err.println("[ResolvedFixtureDump] driverLocationOverride=" + driverLocation);

        ObjectMapper om = new ObjectMapper();

        AutoParser.ParsedDefinitions defs = AutoParser.LoadDefinitions(file);
        AutoSeasonDefinition def = defs.def;

        if (debug) System.err.println("[ResolvedFixtureDump] resolving fixtures...");
        FixtureResolver.ResolveAll(def, allianceTransform, debug);
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

            ObjectNode one = om.createObjectNode();
            one.put("x", x);
            one.put("y", y);
            one.put("units", t.path("positionUnits").asText("meters"));

            // NEW: include rotation if present (already normalized to radians by FixtureResolver)
            if (t.hasNonNull("rotation")) {
                one.put("rot", t.get("rotation").asDouble(0.0));
                one.put("rotUnits", t.path("rotationUnits").asText("radians"));
            }

            fixturesOut.set(e.getKey(), one);
        }

        System.out.println(om.writeValueAsString(out));
    }
}
