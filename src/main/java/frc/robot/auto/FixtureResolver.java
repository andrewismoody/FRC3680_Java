package frc.robot.auto;

import java.util.HashMap;
import java.util.HashSet;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.misc.Utility;

/**
 * Resolves fixtureref objects embedded inside fixtures.
 *
 * fixtureref shape:
 * { "type": string, "index": int }
 */
public final class FixtureResolver {
    private FixtureResolver() {}

    public static void ResolveAll(AutoSeasonDefinition def) {
        ResolveAll(def, true, false);
    }

    public static void ResolveAll(AutoSeasonDefinition def, boolean applyAllianceTransform) {
        ResolveAll(def, applyAllianceTransform, false);
    }

    // NEW: debug-enabled entrypoint
    public static void ResolveAll(AutoSeasonDefinition def, boolean applyAllianceTransform, boolean debug) {
        HashMap<String, JsonNode> byKey = new HashMap<>();

        for (JsonNode f : def.fixtures) {
            String type = f.path("type").asText("");
            int index = f.path("index").asInt(-1);
            byKey.put(key(type, index), f);
        }

        def.resolvedFixtures.clear();

        for (JsonNode f : def.fixtures) {
            String type = f.path("type").asText("");
            int index = f.path("index").asInt(-1);
            String k = key(type, index);

            if (debug) System.err.println("[FixtureResolver] resolving " + k);

            JsonNode resolved = resolveFixture(byKey, k, new HashSet<>(), debug);
            resolved = materializeTranslation(byKey, resolved, new HashSet<>(), applyAllianceTransform, debug);

            def.resolvedFixtures.put(k, resolved);
        }
    }

    private static JsonNode resolveFixture(HashMap<String, JsonNode> byKey, String key, HashSet<String> visiting, boolean debug) {
        JsonNode raw = byKey.get(key);
        if (raw == null) return null;
        if (visiting.contains(key)) {
            if (debug) System.err.println("[FixtureResolver] cycle " + key);
            return raw; // cycle guard
        }

        visiting.add(key);

        // deep copy via tree copy
        JsonNode copy = raw.deepCopy();
        resolveNodeInPlace(byKey, copy, visiting, debug);

        visiting.remove(key);
        return copy;
    }

    private static void resolveNodeInPlace(HashMap<String, JsonNode> byKey, JsonNode node, HashSet<String> visiting, boolean debug) {
        if (node == null) return;

        if (node.isObject()) {
            ObjectNode obj = (ObjectNode) node;

            // iterate fields; replace if fixtureref
            var it = obj.fields();
            while (it.hasNext()) {
                var entry = it.next();
                String k = entry.getKey();
                JsonNode child = entry.getValue();

                JsonNode replacement = resolveMaybeFixtureRef(byKey, child, visiting, debug);
                if (replacement != child) obj.set(k, replacement);

                resolveNodeInPlace(byKey, obj.get(k), visiting, debug);
            }
        } else if (node.isArray()) {
            ArrayNode arr = (ArrayNode) node;
            for (int i = 0; i < arr.size(); i++) {
                JsonNode child = arr.get(i);

                JsonNode replacement = resolveMaybeFixtureRef(byKey, child, visiting, debug);
                if (replacement != child) arr.set(i, replacement);

                resolveNodeInPlace(byKey, arr.get(i), visiting, debug);
            }
        }
    }

    private static JsonNode resolveMaybeFixtureRef(HashMap<String, JsonNode> byKey, JsonNode node, HashSet<String> visiting, boolean debug) {
        if (node == null || !node.isObject()) return node;

        JsonNode typeNode = node.get("type");
        JsonNode indexNode = node.get("index");
        if (typeNode == null || indexNode == null) return node;
        if (!typeNode.isTextual() || !indexNode.isInt()) return node;

        String k = key(typeNode.asText(), indexNode.asInt());
        JsonNode resolved = resolveFixture(byKey, k, visiting, debug);
        return (resolved != null) ? resolved : node;
    }

    private static JsonNode materializeTranslation(
            HashMap<String, JsonNode> byKey,
            JsonNode fixtureNode,
            HashSet<String> visiting,
            boolean applyAllianceTransform,
            boolean debug) {

        if (fixtureNode == null || !fixtureNode.isObject()) return fixtureNode;

        ObjectNode obj = (ObjectNode) fixtureNode;

        // If translation exists, normalize. Apply alliance transform ONLY at the end.
        if (obj.hasNonNull("translation")) {
            Pose3d tr = parsePoseFromSchema(obj.get("translation"));
            if (tr != null) {
                Pose3d out = tr; // CHANGED: do not transform here yet
                if (applyAllianceTransform) out = toAllianceStart(out); // FINAL-ONLY (top-level call path)
                obj.set("translation", toTranslationSchemaNode(obj, out));
                if (debug) System.err.println("[FixtureResolver] direct translation -> " + fmtPos(out.getTranslation()));
            }
            return obj;
        }

        if (!obj.hasNonNull("derivedFrom")) return obj;

        // CHANGED: compute derived pose in BLUE frame regardless of alliance
        Pose3d derivedBlue = evalDerivedFrom(byKey, obj.get("derivedFrom"), visiting, /*applyAllianceTransform=*/false, debug);
        if (derivedBlue != null) {
            Pose3d out = derivedBlue;
            if (applyAllianceTransform) out = toAllianceStart(out); // APPLY ONCE HERE
            obj.set("translation", toTranslationSchemaNode(obj, out));
            if (debug) System.err.println("[FixtureResolver] derived translation -> " + fmtPos(out.getTranslation()));
        }

        return obj;
    }

    private static Pose3d toAllianceStart(Pose3d pose) {
        if (pose == null) return null;

        Translation3d pos = Utility.transformToAllianceStart(pose.getTranslation());
        // keep 2D rotation semantics; treat Pose3d rotation as yaw only
        Rotation2d yaw = Utility.rotateToAllianceStart(pose.getRotation().toRotation2d());

        return new Pose3d(pos, new Rotation3d(0.0, 0.0, yaw.getRadians()));
    }

    private static Pose3d parsePoseFromSchema(JsonNode t) {
        if (t == null || t.isNull()) return null;

        Translation3d posOut = null;
        Rotation2d yawOut = Rotation2d.kZero;

        if (t.hasNonNull("position") && t.get("position").isArray()) {
            JsonNode pos = t.get("position");

            // CHANGED: allow numbers OR expression strings in position elements
            Double x0 = (pos.size() > 0)
                    ? AutoExpr.evalNode(pos.get(0),
                        (name) -> {
                            double v = Utility.GetSeasonNumber(name, Double.NaN);
                            return Double.isNaN(v) ? null : v;
                        },
                        (name, comp) -> {
                            Translation2d v2 = Utility.GetSeasonVec2Inches(name, null);
                            if (v2 == null) return null;
                            return (comp == 'X') ? v2.getX() : (comp == 'Y' ? v2.getY() : null);
                        },
                        (name, comp) -> {
                            Translation3d v3 = Utility.GetSeasonVec3Inches(name, null);
                            if (v3 == null) return null;
                            if (comp == 'X') return v3.getX();
                            if (comp == 'Y') return v3.getY();
                            if (comp == 'Z') return v3.getZ();
                            return null;
                        })
                    : null;

            Double y0 = (pos.size() > 1)
                    ? AutoExpr.evalNode(pos.get(1),
                        (name) -> {
                            double v = Utility.GetSeasonNumber(name, Double.NaN);
                            return Double.isNaN(v) ? null : v;
                        },
                        (name, comp) -> {
                            Translation2d v2 = Utility.GetSeasonVec2Inches(name, null);
                            if (v2 == null) return null;
                            return (comp == 'X') ? v2.getX() : (comp == 'Y' ? v2.getY() : null);
                        },
                        (name, comp) -> {
                            Translation3d v3 = Utility.GetSeasonVec3Inches(name, null);
                            if (v3 == null) return null;
                            if (comp == 'X') return v3.getX();
                            if (comp == 'Y') return v3.getY();
                            if (comp == 'Z') return v3.getZ();
                            return null;
                        })
                    : null;

            Double z0 = (pos.size() > 2)
                    ? AutoExpr.evalNode(pos.get(2),
                        (name) -> {
                            double v = Utility.GetSeasonNumber(name, Double.NaN);
                            return Double.isNaN(v) ? null : v;
                        },
                        (name, comp) -> {
                            Translation2d v2 = Utility.GetSeasonVec2Inches(name, null);
                            if (v2 == null) return null;
                            return (comp == 'X') ? v2.getX() : (comp == 'Y' ? v2.getY() : null);
                        },
                        (name, comp) -> {
                            Translation3d v3 = Utility.GetSeasonVec3Inches(name, null);
                            if (v3 == null) return null;
                            if (comp == 'X') return v3.getX();
                            if (comp == 'Y') return v3.getY();
                            if (comp == 'Z') return v3.getZ();
                            return null;
                        })
                    : 0.0;

            double x = (x0 != null) ? x0.doubleValue() : 0.0;
            double y = (y0 != null) ? y0.doubleValue() : 0.0;
            double z = (z0 != null) ? z0.doubleValue() : 0.0;

            String units = t.path("positionUnits").asText("inches");
            if ("inches".equalsIgnoreCase(units)) {
                x = Utility.inchesToMeters(x);
                y = Utility.inchesToMeters(y);
                z = Utility.inchesToMeters(z);
            }
            posOut = new Translation3d(x, y, z);
        }

        if (t.hasNonNull("rotation")) {
            double r = t.get("rotation").asDouble(0.0);
            String units = t.path("rotationUnits").asText("degrees");
            if ("degrees".equalsIgnoreCase(units)) r = Utility.degreesToRadians(r);
            yawOut = new Rotation2d(r);
        }

        if (posOut == null) return null;
        return new Pose3d(posOut, new Rotation3d(yawOut));
    }

    private static ObjectNode toTranslationSchemaNode(ObjectNode owner, Pose3d pose) {
        ObjectNode t = owner.objectNode();

        Translation3d p = pose.getTranslation();
        ArrayNode pos = owner.arrayNode();
        pos.add(p.getX());
        pos.add(p.getY());
        pos.add(p.getZ());
        t.set("position", pos);
        t.put("positionUnits", "meters");

        // always emit yaw rotation (radians)
        t.put("rotation", pose.getRotation().toRotation2d().getRadians());
        t.put("rotationUnits", "radians");

        return t;
    }

    private static Pose3d evalDerivedFrom(
            HashMap<String, JsonNode> byKey,
            JsonNode d,
            HashSet<String> visiting,
            boolean applyAllianceTransform,
            boolean debug) {

        // NOTE:
        // applyAllianceTransform is intentionally ignored for derived computation now.
        // Derived computation stays in BLUE frame; top-level materializeTranslation applies transform once.

        if (d == null || d.isNull()) return null;

        String function = d.path("function").asText("none");
        function = (function == null) ? "none" : function.trim().toLowerCase(); // CHANGED

        double offsetInches = d.path("offset").asDouble(0.0);
        double offset = Utility.inchesToMeters(offsetInches);

        if (debug) {
            System.err.println("[FixtureResolver]   fn=" + function
                    + " offset(in)=" + offsetInches
                    + " offset(m)=" + String.format("%.4f", offset));
        }

        // CHANGED: always resolve args in BLUE frame (no alliance transform during recursion)
        Pose3d a = resolveDerivedArgPose(byKey, d.get("fixture"), d.get("derivedFrom"), visiting, /*applyAllianceTransform=*/false, debug);
        Pose3d b = resolveDerivedArgPose(byKey, d.get("fixture2"), d.get("derivedFrom2"), visiting, /*applyAllianceTransform=*/false, debug);

        switch (function) {
            case "none":
                return a;

            case "parallel": {
                if (a == null) return null;
                Rotation2d base = a.getRotation().toRotation2d();
                // subtract 90 degrees from any direct values because the field is rotated by 90
                Rotation2d ref = base.minus(Rotation2d.fromDegrees(90.0));
                Translation3d pos = Utility.projectParallel(a.getTranslation(), ref, offset);
                Pose3d out = new Pose3d(pos, new Rotation3d(base));
                if (debug) System.err.println("[FixtureResolver]     parallel from=" + fmtPos(a.getTranslation()) + " -> " + fmtPos(out.getTranslation()));
                return out;
            }

            case "perpendicular": {
                if (a == null) return null;
                Rotation2d base = a.getRotation().toRotation2d();
                // subtract 90 degrees from any direct values because the field is rotated by 90
                Rotation2d ref = base.minus(Rotation2d.fromDegrees(90.0));
                Translation3d pos = Utility.projectPerpendicular(a.getTranslation(), ref, offset);
                Pose3d out = new Pose3d(pos, new Rotation3d(base));
                if (debug) System.err.println("[FixtureResolver]     perpendicular from=" + fmtPos(a.getTranslation()) + " -> " + fmtPos(out.getTranslation()));
                return out;
            }

            case "bisector": {
                if (a == null || b == null) return null;

                // subtract 90 degrees from any direct values because the field is rotated by 90
                var a2 = new Pose3d(a.getTranslation(), new Rotation3d(a.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90.0))));
                var b2 = new Pose3d(b.getTranslation(), new Rotation3d(b.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90.0))));

                Pose2d bis = Utility.perpendicularBisectorAngle(
                        a2.getTranslation().toTranslation2d(),
                        b2.getTranslation().toTranslation2d());

                var inter = Utility.getIntersection(a2.toPose2d(), b2.toPose2d());
                Translation3d base = new Translation3d(inter);
                Pose3d out = new Pose3d(base, new Rotation3d(bis.getRotation().plus(Rotation2d.fromDegrees(90.0))));
                if (debug) System.err.println("[FixtureResolver]     bisector a=" + fmtPos(a.getTranslation()) + " b=" + fmtPos(b.getTranslation()) + " -> " + fmtPos(out.getTranslation()));
                return out;
            }

            default:
                return a;
        }
    }

    private static Pose3d resolveDerivedArgPose(
            HashMap<String, JsonNode> byKey,
            JsonNode fixtureRef,
            JsonNode derivedFrom,
            HashSet<String> visiting,
            boolean applyAllianceTransform,
            boolean debug) {

        // CHANGED: during derived evaluation, we never alliance-transform inputs
        final boolean recurseAllianceTransform = false;

        if (fixtureRef != null && fixtureRef.isObject()) {
            String type = fixtureRef.path("type").asText("");
            int index = fixtureRef.path("index").asInt(-1);
            if (index >= 0) {
                JsonNode f = byKey.get(key(type, index));
                if (f != null) {
                    JsonNode copy = f.deepCopy();
                    resolveNodeInPlace(byKey, copy, visiting, debug);
                    materializeTranslation(byKey, copy, visiting, recurseAllianceTransform, debug); // CHANGED
                    if (copy.hasNonNull("translation")) {
                        return parsePoseFromSchema(copy.get("translation"));
                    }
                }
            }
        }

        if (derivedFrom != null && !derivedFrom.isNull()) {
            return evalDerivedFrom(byKey, derivedFrom, visiting, recurseAllianceTransform, debug); // CHANGED
        }

        return null;
    }

    private static String key(String type, int index) {
        return type + ":" + index;
    }

    private static String fmtPos(Translation3d t) {
        if (t == null) return "null";
        return String.format(
                "m=(%.3f,%.3f,%.3f) in=(%.1f,%.1f,%.1f)",
                t.getX(), t.getY(), t.getZ(),
                Utility.metersToInches(t.getX()),
                Utility.metersToInches(t.getY()),
                Utility.metersToInches(t.getZ()));
    }
}
