package frc.robot.auto;

import java.util.HashMap;
import java.util.HashSet;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        HashMap<String, JsonNode> byKey = new HashMap<>();

        int idx = 0;
        for (JsonNode f : def.fixtures) {
            String type = f.path("type").asText("");
            byKey.put(key(type, idx), f);
            idx++;
        }

        def.resolvedFixtures.clear();

        idx = 0;
        for (JsonNode f : def.fixtures) {
            String type = f.path("type").asText("");
            String k = key(type, idx);

            JsonNode resolved = resolveFixture(byKey, k, new HashSet<>());
            resolved = materializeTranslation(byKey, resolved, new HashSet<>());

            def.resolvedFixtures.put(k, resolved);
            idx++;
        }
    }

    private static JsonNode resolveFixture(HashMap<String, JsonNode> byKey, String key, HashSet<String> visiting) {
        JsonNode raw = byKey.get(key);
        if (raw == null) return null;
        if (visiting.contains(key)) return raw; // cycle guard

        visiting.add(key);

        // deep copy via tree copy
        JsonNode copy = raw.deepCopy();
        resolveNodeInPlace(byKey, copy, visiting);

        visiting.remove(key);
        return copy;
    }

    private static void resolveNodeInPlace(HashMap<String, JsonNode> byKey, JsonNode node, HashSet<String> visiting) {
        if (node == null) return;

        if (node.isObject()) {
            ObjectNode obj = (ObjectNode) node;

            // iterate fields; replace if fixtureref
            var it = obj.fields();
            while (it.hasNext()) {
                var entry = it.next();
                String k = entry.getKey();
                JsonNode child = entry.getValue();

                JsonNode replacement = resolveMaybeFixtureRef(byKey, child, visiting);
                if (replacement != child) obj.set(k, replacement);

                resolveNodeInPlace(byKey, obj.get(k), visiting);
            }
        } else if (node.isArray()) {
            ArrayNode arr = (ArrayNode) node;
            for (int i = 0; i < arr.size(); i++) {
                JsonNode child = arr.get(i);

                JsonNode replacement = resolveMaybeFixtureRef(byKey, child, visiting);
                if (replacement != child) arr.set(i, replacement);

                resolveNodeInPlace(byKey, arr.get(i), visiting);
            }
        }
    }

    private static JsonNode resolveMaybeFixtureRef(HashMap<String, JsonNode> byKey, JsonNode node, HashSet<String> visiting) {
        if (node == null || !node.isObject()) return node;

        JsonNode typeNode = node.get("type");
        JsonNode indexNode = node.get("index");
        if (typeNode == null || indexNode == null) return node;
        if (!typeNode.isTextual() || !indexNode.isInt()) return node;

        String k = key(typeNode.asText(), indexNode.asInt());
        JsonNode resolved = resolveFixture(byKey, k, visiting);
        return (resolved != null) ? resolved : node;
    }

    private static JsonNode materializeTranslation(HashMap<String, JsonNode> byKey, JsonNode fixtureNode, HashSet<String> visiting) {
        if (fixtureNode == null || !fixtureNode.isObject()) return fixtureNode;

        ObjectNode obj = (ObjectNode) fixtureNode;

        // FIX: if translation exists, normalize it to schema format in meters + radians
        if (obj.hasNonNull("translation")) {
            TranslationRotation tr = parseTranslationFromSchema(obj.get("translation"));
            if (tr != null) obj.set("translation", toTranslationSchemaNode(obj, tr));
            return obj;
        }

        if (!obj.hasNonNull("derivedFrom")) return obj;

        Translation3d pos = evalDerivedFrom(byKey, obj.get("derivedFrom"), visiting);
        if (pos != null) {
            // derivedFrom only defines translation; rotation remains unset
            obj.set("translation", toTranslationSchemaNode(obj, new TranslationRotation(pos, null)));
        }

        return obj;
    }

    // NEW: holder for position+rotation (rotation optional)
    private static final class TranslationRotation {
        final Translation3d position;
        final Rotation2d rotation; // nullable
        TranslationRotation(Translation3d position, Rotation2d rotation) {
            this.position = position;
            this.rotation = rotation;
        }
    }

    // NEW: translation.schema.json parser (position + rotation, with units)
    private static TranslationRotation parseTranslationFromSchema(JsonNode t) {
        if (t == null || t.isNull()) return null;

        Translation3d posOut = null;
        Rotation2d rotOut = null;

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
            rotOut = new Rotation2d(r);
        }

        if (posOut == null && rotOut == null) return null;
        return new TranslationRotation(posOut, rotOut);
    }

    private static ObjectNode toTranslationSchemaNode(ObjectNode owner, TranslationRotation tr) {
        ObjectNode t = owner.objectNode();

        if (tr.position != null) {
            ArrayNode pos = owner.arrayNode();
            pos.add(tr.position.getX());
            pos.add(tr.position.getY());
            pos.add(tr.position.getZ());
            t.set("position", pos);
            t.put("positionUnits", "meters");
        }

        if (tr.rotation != null) {
            t.put("rotation", tr.rotation.getRadians());
            t.put("rotationUnits", "radians");
        }

        return t;
    }

    private static Translation3d parseTranslation3dFromSchema(JsonNode t) {
        TranslationRotation tr = parseTranslationFromSchema(t);
        return (tr != null) ? tr.position : null;
    }

    private static Translation3d evalDerivedFrom(HashMap<String, JsonNode> byKey, JsonNode d, HashSet<String> visiting) {
        if (d == null || d.isNull()) return null;

        String function = d.path("function").asText("none");
        double offset = d.path("offset").asDouble(0.0);

        Translation3d a = resolveDerivedArg(byKey, d.get("fixture"), d.get("derivedFrom"), visiting);
        Translation3d b = resolveDerivedArg(byKey, d.get("fixture2"), d.get("derivedFrom2"), visiting);

        switch (function) {
            case "none": {
                return a; // offset ignored for none (keep minimal/deterministic)
            }

            case "parallel": {
                if (a == null || b == null) return a;
                Rotation2d angle = Utility.getLookat(a.toTranslation2d(), b.toTranslation2d());
                return Utility.projectParallel(a, angle, offset);
            }

            case "perpendicular": {
                if (a == null || b == null) return a;
                Rotation2d angle = Utility.getLookat(a.toTranslation2d(), b.toTranslation2d());
                return Utility.projectPerpendicular(a, angle, offset);
            }

            case "bisector": {
                if (a == null || b == null) return null;

                // Perpendicular bisector of segment AB; get a pose anchored at the midpoint with bisector rotation
                Pose2d bis = Utility.perpendicularBisectorAngle(a.toTranslation2d(), b.toTranslation2d());

                // We need a second bisector line to intersect with for a stable point.
                // Minimal deterministic choice: a line through A pointing along the bisector rotation.
                Pose2d throughA = new Pose2d(a.toTranslation2d(), bis.getRotation());

                Translation2d intersect = Utility.getIntersection(bis, throughA);
                Translation3d base = new Translation3d(intersect);

                // Apply offset parallel to AB (keeps offset semantics consistent)
                Rotation2d along = Utility.getLookat(a.toTranslation2d(), b.toTranslation2d());
                return Utility.projectParallel(base, along, offset);
            }

            default:
                return a;
        }
    }

    private static Translation3d resolveDerivedArg(
            HashMap<String, JsonNode> byKey,
            JsonNode fixtureRef,
            JsonNode derivedFrom,
            HashSet<String> visiting) {

        if (fixtureRef != null && fixtureRef.isObject()) {
            String type = fixtureRef.path("type").asText("");
            int index = fixtureRef.path("index").asInt(-1);
            if (index >= 0) {
                JsonNode f = byKey.get(key(type, index));
                if (f != null) {
                    JsonNode copy = f.deepCopy();
                    resolveNodeInPlace(byKey, copy, visiting);
                    materializeTranslation(byKey, copy, visiting);
                    if (copy.hasNonNull("translation")) {
                        // FIX: translation is schema-shaped
                        return parseTranslation3dFromSchema(copy.get("translation"));
                    }
                }
            }
        }

        if (derivedFrom != null && !derivedFrom.isNull()) {
            return evalDerivedFrom(byKey, derivedFrom, visiting);
        }

        return null;
    }

    private static String key(String type, int index) {
        return type + ":" + index;
    }
}
