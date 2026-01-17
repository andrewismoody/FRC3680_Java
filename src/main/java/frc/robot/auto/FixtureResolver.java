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

            JsonNode resolved = resolveFixture(byKey, k, new HashSet<>());
            resolved = materializeTranslation(byKey, resolved, new HashSet<>());

            def.resolvedFixtures.put(k, resolved);
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

        // If translation exists, normalize AND transform to alliance-start coordinates.
        if (obj.hasNonNull("translation")) {
            TranslationRotation tr = parseTranslationFromSchema(obj.get("translation"));
            if (tr != null) obj.set("translation", toTranslationSchemaNode(obj, toAllianceStart(tr)));
            return obj;
        }

        if (!obj.hasNonNull("derivedFrom")) return obj;

        Translation3d pos = evalDerivedFrom(byKey, obj.get("derivedFrom"), visiting);
        if (pos != null) {
            // derivedFrom only defines translation; rotation remains unset
            obj.set("translation", toTranslationSchemaNode(obj, toAllianceStart(new TranslationRotation(pos, null))));
        }

        return obj;
    }

    // NEW: apply already-available alliance transforms, without changing JSON/schema.
    private static TranslationRotation toAllianceStart(TranslationRotation tr) {
        if (tr == null) return null;

        Translation3d pos = tr.position;
        Rotation2d rot = tr.rotation;

        if (pos != null) pos = Utility.transformToAllianceStart(pos);
        if (rot != null) rot = Utility.rotateToAllianceStart(rot);

        return new TranslationRotation(pos, rot);
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

    private static Translation3d evalDerivedFrom(HashMap<String, JsonNode> byKey, JsonNode d, HashSet<String> visiting) {
        if (d == null || d.isNull()) return null;

        String function = d.path("function").asText("none");
        double offset = d.path("offset").asDouble(0.0);

        // CHANGE: we need rotation for parallel/perpendicular; only bisector needs a second arg
        TranslationRotation a = resolveDerivedArgTR(byKey, d.get("fixture"), d.get("derivedFrom"), visiting);
        TranslationRotation b = resolveDerivedArgTR(byKey, d.get("fixture2"), d.get("derivedFrom2"), visiting);

        switch (function) {
            case "none": {
                return (a != null) ? a.position : null;
            }

            case "parallel": {
                if (a == null || a.position == null) return null;
                Rotation2d angle = (a.rotation != null) ? a.rotation : Rotation2d.kZero;
                return Utility.projectParallel(a.position, angle, offset);
            }

            case "perpendicular": {
                if (a == null || a.position == null) return null;
                Rotation2d angle = (a.rotation != null) ? a.rotation : Rotation2d.kZero;
                return Utility.projectPerpendicular(a.position, angle, offset);
            }

            case "bisector": {
                if (a == null || b == null || a.position == null || b.position == null) return null;

                Pose2d bis = Utility.perpendicularBisectorAngle(a.position.toTranslation2d(), b.position.toTranslation2d());
                Pose2d throughA = new Pose2d(a.position.toTranslation2d(), bis.getRotation());

                Translation2d intersect = Utility.getIntersection(bis, throughA);
                Translation3d base = new Translation3d(intersect);

                Rotation2d along = Utility.getLookat(a.position.toTranslation2d(), b.position.toTranslation2d());
                return Utility.projectParallel(base, along, offset);
            }

            default:
                return (a != null) ? a.position : null;
        }
    }

    // NEW: like resolveDerivedArg, but preserves rotation too
    private static TranslationRotation resolveDerivedArgTR(
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
                        return parseTranslationFromSchema(copy.get("translation"));
                    }
                }
            }
        }

        if (derivedFrom != null && !derivedFrom.isNull()) {
            Translation3d pos = evalDerivedFrom(byKey, derivedFrom, visiting);
            // derivedFrom currently only yields a position; rotation is not defined
            return new TranslationRotation(pos, null);
        }

        return null;
    }

    private static String key(String type, int index) {
        return type + ":" + index;
    }
}
