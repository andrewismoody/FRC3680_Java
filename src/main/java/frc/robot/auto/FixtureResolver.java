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

        // If translation exists, normalize AND (optionally) transform to alliance-start coordinates.
        if (obj.hasNonNull("translation")) {
            Pose3d tr = parsePoseFromSchema(obj.get("translation"));
            if (tr != null) {
                Pose3d out = applyAllianceTransform ? toAllianceStart(tr) : tr;
                obj.set("translation", toTranslationSchemaNode(obj, out));
                if (debug) System.err.println("[FixtureResolver] direct translation -> " + out.getTranslation());
            }
            return obj;
        }

        if (!obj.hasNonNull("derivedFrom")) return obj;

        Pose3d derived = evalDerivedFrom(byKey, obj.get("derivedFrom"), visiting, applyAllianceTransform, debug);
        if (derived != null) {
            Pose3d out = applyAllianceTransform ? toAllianceStart(derived) : derived;
            obj.set("translation", toTranslationSchemaNode(obj, out));
            if (debug) System.err.println("[FixtureResolver] derived translation -> " + out.getTranslation());
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
            double x = pos.size() > 0 ? pos.get(0).asDouble(0.0) : 0.0;
            double y = pos.size() > 1 ? pos.get(1).asDouble(0.0) : 0.0;
            double z = pos.size() > 2 ? pos.get(2).asDouble(0.0) : 0.0;

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
        return new Pose3d(posOut, new Rotation3d(0.0, 0.0, yawOut.getRadians()));
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

        if (d == null || d.isNull()) return null;

        String function = d.path("function").asText("none");
        function = (function == null) ? "none" : function.trim().toLowerCase(); // CHANGED

        double offsetInches = d.path("offset").asDouble(0.0);
        double offset = Utility.inchesToMeters(offsetInches);

        if (debug) System.err.println("[FixtureResolver]   fn=" + function + " offset(in)=" + offsetInches);

        Pose3d a = resolveDerivedArgPose(byKey, d.get("fixture"), d.get("derivedFrom"), visiting, applyAllianceTransform, debug);
        Pose3d b = resolveDerivedArgPose(byKey, d.get("fixture2"), d.get("derivedFrom2"), visiting, applyAllianceTransform, debug);

        switch (function) {
            case "none":
                return a;

            case "parallel": {
                if (a == null) return null;
                Rotation2d base = a.getRotation().toRotation2d();
                Rotation2d ref = base.minus(Rotation2d.fromDegrees(90.0));
                Translation3d pos = Utility.projectParallel(a.getTranslation(), ref, offset);
                Pose3d out = new Pose3d(pos, new Rotation3d(ref));
                if (debug) System.err.println("[FixtureResolver]     parallel from=" + a.getTranslation() + " -> " + out.getTranslation());
                return out;
            }

            case "perpendicular": {
                if (a == null) return null;
                Rotation2d base = a.getRotation().toRotation2d();
                Rotation2d ref = base.minus(Rotation2d.fromDegrees(90.0));
                Rotation2d ang = ref.plus(Rotation2d.fromDegrees(90.0));
                Translation3d pos = Utility.projectParallel(a.getTranslation(), ang, offset);
                Pose3d out = new Pose3d(pos, new Rotation3d(ang));
                if (debug) System.err.println("[FixtureResolver]     perpendicular from=" + a.getTranslation() + " -> " + out.getTranslation());
                return out;
            }

            case "bisector": {
                if (a == null || b == null) return null;

                Pose2d bis = Utility.perpendicularBisectorAngle(
                        a.getTranslation().toTranslation2d(),
                        b.getTranslation().toTranslation2d());

                Translation3d base = new Translation3d(bis.getTranslation());
                Pose3d out = new Pose3d(base, new Rotation3d(bis.getRotation()));
                if (debug) System.err.println("[FixtureResolver]     bisector -> " + out.getTranslation());
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

        if (fixtureRef != null && fixtureRef.isObject()) {
            String type = fixtureRef.path("type").asText("");
            int index = fixtureRef.path("index").asInt(-1);
            if (index >= 0) {
                JsonNode f = byKey.get(key(type, index));
                if (f != null) {
                    JsonNode copy = f.deepCopy();
                    resolveNodeInPlace(byKey, copy, visiting, debug);
                    materializeTranslation(byKey, copy, visiting, applyAllianceTransform, debug);
                    if (copy.hasNonNull("translation")) {
                        return parsePoseFromSchema(copy.get("translation"));
                    }
                }
            }
        }

        if (derivedFrom != null && !derivedFrom.isNull()) {
            return evalDerivedFrom(byKey, derivedFrom, visiting, applyAllianceTransform, debug);
        }

        return null;
    }

    private static String key(String type, int index) {
        return type + ":" + index;
    }
}
