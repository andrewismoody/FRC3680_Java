package frc.robot.auto;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.misc.Utility;

import frc.robot.modules.ModuleController;

/**
 * Minimal season-auto JSON parser.
 *
 * Notes:
 * - Trusts JSON structure per schema. Avoids coupling to predefined enums.
 * - Loads vocab label arrays into AutoController for later use.
 * - Sequence instantiation is intentionally minimal until a concrete builder is confirmed.
 */
public final class AutoParser {

    private static final ObjectMapper MAPPER = new ObjectMapper();

    private AutoParser() {}

    public static final class ParsedDefinitions {
        public final AutoSeasonDefinition def;

        public final JsonNode root;

        private ParsedDefinitions(AutoSeasonDefinition def, JsonNode root) {
            this.def = def;
            this.root = root;
        }
    }

    public static ParsedDefinitions LoadDefinitions(String filePath) throws IOException {
        JsonNode root = MAPPER.readTree(Files.readString(Path.of(filePath)));

        String season = reqText(root, "season");
        String version = reqText(root, "version");
        String description = reqText(root, "description");

        ArrayList<String> groups = toStringList(reqArray(root, "groups"));
        ArrayList<String> locations = toStringList(reqArray(root, "locations"));
        ArrayList<String> positions = toStringList(reqArray(root, "positions"));
        ArrayList<String> actions = toStringList(reqArray(root, "actions"));

        ArrayList<String> travelGroups = root.has("travelGroups") && root.get("travelGroups").isArray()
                ? toStringList(root.get("travelGroups"))
                : new ArrayList<>();

        java.util.Map<String, JsonNode> rawParams = new java.util.HashMap<>();
        if (root.hasNonNull("params") && root.get("params").isObject()) {
            var it = root.get("params").fields();
            while (it.hasNext()) {
                var e = it.next();
                rawParams.put(e.getKey(), e.getValue());
            }
        }

        HashMap<String, Double> params = resolveParams(rawParams);

        HashMap<String, Translation2d> vec2Params = resolveVec2Params(rawParams, params);
        // CHANGED: pass vec2Params into vec3 resolver so vec3 can reference vec2 expressions
        HashMap<String, Translation3d> vec3Params = resolveVec3Params(rawParams, params, vec2Params);

        Utility.SetSeasonVec2Params(vec2Params);
        Utility.SetSeasonVec3Params(vec3Params);

        if (root.isObject()) {
            expandOffsetVars((ObjectNode) root, params, vec2Params);
        }

        JsonNode poses = reqArray(root, "poses");
        JsonNode fixtures = reqArray(root, "fixtures");
        JsonNode events = reqArray(root, "events");
        JsonNode targets = reqArray(root, "targets");
        JsonNode sequences = reqArray(root, "sequences");

        AutoSeasonDefinition def = new AutoSeasonDefinition(
                root, season, version, description,
                groups, locations, positions, actions,
                poses, fixtures, events, targets, sequences);

        def.travelGroups = travelGroups;
        def.params.putAll(params);

        return new ParsedDefinitions(def, root);
    }

    // NEW: preload only params/vec2/vec3 into Utility (no fixtures resolved, no controller wiring)
    public static ParsedDefinitions PreloadSeasonParams(String filePath) throws IOException {
        ParsedDefinitions defs = LoadDefinitions(filePath);

        // Push resolved params into Utility so Utility can self-configure without calling LoadIntoController.
        AutoSeasonDefinition def = defs.def;
        Utility.SetSeasonParams(def.params);

        return defs;
    }

    // NEW: finish building controller using a preloaded/parsed definition
    public static void LoadIntoControllerFromPreloaded(
            ParsedDefinitions defs,
            AutoController controller,
            ModuleController moduleController,
            Supplier<Integer> startLocationProvider,
            boolean applyAllianceTransform) {

        if (defs == null || defs.def == null) {
            throw new IllegalArgumentException("defs/defs.def is null");
        }

        AutoSeasonDefinition def = defs.def;

        controller.SetDefinitions(def.groups, def.locations, def.positions, def.actions);

        FixtureResolver.ResolveAll(def, applyAllianceTransform);

        controller.SetSeasonDefinition(def);

        AutoSequenceFactory factory = new AutoSequenceFactory(controller, def, startLocationProvider);
        for (JsonNode seqNode : def.sequences) {
            AutoSequence seq = factory.fromSequenceJson(seqNode);
            if (seq != null) controller.AddSequence(seq);
        }
    }

    public static Translation2d GetFieldSizeInchesFromDefinitions(AutoSeasonDefinition def) {
        if (def == null || def.root == null) return null;

        JsonNode paramsNode = def.root.get("params");
        if (paramsNode == null || !paramsNode.isObject()) return null;

        JsonNode fieldSize = paramsNode.get("fieldSize");
        if (fieldSize == null || !fieldSize.isArray() || fieldSize.size() < 2) return null;

        Double x = AutoExpr.evalNode(
                fieldSize.get(0),
                def.params::get,
                (name, comp) -> {
                    Translation2d v = Utility.GetSeasonVec2Inches(name, null);
                    if (v == null) return null;
                    return (comp == 'X') ? v.getX() : (comp == 'Y' ? v.getY() : null);
                },
                (name, comp) -> {
                    Translation3d v = Utility.GetSeasonVec3Inches(name, null);
                    if (v == null) return null;
                    if (comp == 'X') return v.getX();
                    if (comp == 'Y') return v.getY();
                    if (comp == 'Z') return v.getZ();
                    return null;
                });

        Double y = AutoExpr.evalNode(
                fieldSize.get(1),
                def.params::get,
                (name, comp) -> {
                    Translation2d v = Utility.GetSeasonVec2Inches(name, null);
                    if (v == null) return null;
                    return (comp == 'X') ? v.getX() : (comp == 'Y' ? v.getY() : null);
                },
                (name, comp) -> {
                    Translation3d v = Utility.GetSeasonVec3Inches(name, null);
                    if (v == null) return null;
                    if (comp == 'X') return v.getX();
                    if (comp == 'Y') return v.getY();
                    if (comp == 'Z') return v.getZ();
                    return null;
                });

        if (x == null || y == null) return null;
        return new Translation2d(x.doubleValue(), y.doubleValue());
    }

    private static HashMap<String, Double> resolveParams(java.util.Map<String, JsonNode> rawParams) {
        HashMap<String, Double> out = new HashMap<>();
        if (rawParams == null || rawParams.isEmpty()) return out;

        java.util.HashMap<String, Double> memo = new java.util.HashMap<>();
        java.util.HashSet<String> visiting = new java.util.HashSet<>();

        for (String k : rawParams.keySet()) {
            Double v = resolveParamValue(k, rawParams, memo, visiting);
            if (v != null) out.put(k, v.doubleValue());
        }
        return out;
    }

    private static Double resolveParamValue(
            String key,
            java.util.Map<String, JsonNode> rawParams,
            java.util.HashMap<String, Double> memo,
            java.util.HashSet<String> visiting) {

        if (memo.containsKey(key)) return memo.get(key);
        if (visiting.contains(key)) {
            throw new IllegalArgumentException("Param recursion cycle detected at: " + key);
        }

        JsonNode node = rawParams.get(key);
        if (node == null || node.isNull()) return null;

        visiting.add(key);

        Double v = null;

        if (node.isNumber()) {
            v = node.asDouble();
        } else if (node.isTextual()) {
            String expr = node.asText();
            if (expr != null) expr = expr.trim();
            if (expr != null && !expr.isEmpty()) {
                v = AutoExpr.eval(expr, (name) -> resolveParamValue(name, rawParams, memo, visiting), null);
            }
        }

        visiting.remove(key);

        if (v != null) memo.put(key, v);
        return v;
    }

    private static HashMap<String, Translation2d> resolveVec2Params(
            java.util.Map<String, JsonNode> rawParams,
            java.util.Map<String, Double> resolvedScalarParams) {

        HashMap<String, Translation2d> out = new HashMap<>();
        if (rawParams == null || rawParams.isEmpty()) return out;

        java.util.HashMap<String, Translation2d> partial = new java.util.HashMap<>();

        int passes = 3;
        for (int pass = 0; pass < passes; pass++) {
            boolean changed = false;

            for (var e : rawParams.entrySet()) {
                String key = e.getKey();
                JsonNode n = e.getValue();
                if (n == null || !n.isArray() || n.size() < 2) continue;

                Double x = evalNodeToDouble(n.get(0), resolvedScalarParams, partial);
                Double y = evalNodeToDouble(n.get(1), resolvedScalarParams, partial);
                if (x == null || y == null) continue;

                Translation2d v = new Translation2d(x.doubleValue(), y.doubleValue());
                Translation2d prev = partial.get(key);
                if (prev == null || prev.getX() != v.getX() || prev.getY() != v.getY()) {
                    partial.put(key, v);
                    changed = true;
                }
            }

            if (!changed) break;
        }

        out.putAll(partial);
        return out;
    }

    // CHANGED: add resolvedVec2Params parameter
    private static HashMap<String, Translation3d> resolveVec3Params(
            java.util.Map<String, JsonNode> rawParams,
            java.util.Map<String, Double> resolvedScalarParams,
            java.util.Map<String, Translation2d> resolvedVec2Params) {

        HashMap<String, Translation3d> out = new HashMap<>();
        if (rawParams == null || rawParams.isEmpty()) return out;

        // CHANGED: seed vec2Partial with already-resolved vec2s (so $fieldCenter.X works immediately)
        java.util.HashMap<String, Translation2d> vec2Partial = new java.util.HashMap<>();
        if (resolvedVec2Params != null) vec2Partial.putAll(resolvedVec2Params);

        java.util.HashMap<String, Translation3d> vec3Partial = new java.util.HashMap<>();

        int passes = 3;
        for (int pass = 0; pass < passes; pass++) {
            boolean changed = false;

            for (var e : rawParams.entrySet()) {
                String key = e.getKey();
                JsonNode n = e.getValue();
                if (n == null || !n.isArray() || n.size() < 3) continue;

                Double x = evalNodeToDouble(n.get(0), resolvedScalarParams, vec2Partial, vec3Partial);
                Double y = evalNodeToDouble(n.get(1), resolvedScalarParams, vec2Partial, vec3Partial);
                Double z = evalNodeToDouble(n.get(2), resolvedScalarParams, vec2Partial, vec3Partial);
                if (x == null || y == null || z == null) continue;

                Translation3d v = new Translation3d(x.doubleValue(), y.doubleValue(), z.doubleValue());
                Translation3d prev = vec3Partial.get(key);
                if (prev == null || prev.getX() != v.getX() || prev.getY() != v.getY() || prev.getZ() != v.getZ()) {
                    vec3Partial.put(key, v);
                    changed = true;
                }

                // Keep vec2 view updated as a convenience for later vec3 expressions
                vec2Partial.put(key, new Translation2d(v.getX(), v.getY()));
            }

            if (!changed) break;
        }

        out.putAll(vec3Partial);
        return out;
    }

    private static Double evalNodeToDouble(
            JsonNode node,
            java.util.Map<String, Double> scalarParams,
            java.util.Map<String, Translation2d> vec2Params) {

        return AutoExpr.evalNode(
            node,
            scalarParams::get,
            (name, comp) -> {
                Translation2d v2 = (vec2Params != null) ? vec2Params.get(name) : null;
                if (v2 == null) return null;
                return (comp == 'X') ? v2.getX() : (comp == 'Y' ? v2.getY() : null);
            },
            null
        );
    }

    private static Double evalNodeToDouble(
            JsonNode node,
            java.util.Map<String, Double> scalarParams,
            java.util.Map<String, Translation2d> vec2Params,
            java.util.Map<String, Translation3d> vec3Params) {

        return AutoExpr.evalNode(
            node,
            scalarParams::get,
            (name, comp) -> {
                Translation2d v2 = (vec2Params != null) ? vec2Params.get(name) : null;
                if (v2 == null) return null;
                return (comp == 'X') ? v2.getX() : (comp == 'Y' ? v2.getY() : null);
            },
            (name, comp) -> {
                Translation3d v3 = (vec3Params != null) ? vec3Params.get(name) : null;
                if (v3 == null) return null;
                if (comp == 'X') return v3.getX();
                if (comp == 'Y') return v3.getY();
                if (comp == 'Z') return v3.getZ();
                return null;
            }
        );
    }

    private static double evalExpr(
            String expr,
            java.util.Map<String, Double> scalarParams,
            java.util.Map<String, Translation2d> vec2Params) {

        Double v = AutoExpr.eval(
            expr,
            scalarParams::get,
            (name, comp) -> {
                Translation2d v2 = (vec2Params != null) ? vec2Params.get(name) : null;
                if (v2 == null) return null;
                return (comp == 'X') ? v2.getX() : (comp == 'Y' ? v2.getY() : null);
            },
            null
        );

        if (v == null) throw new RuntimeException("Expr eval failed: " + expr);
        return v.doubleValue();
    }

    private static double evalExpr(
            String expr,
            java.util.Map<String, Double> scalarParams,
            java.util.Map<String, Translation2d> vec2Params,
            java.util.Map<String, Translation3d> vec3Params) {

        Double v = AutoExpr.eval(
            expr,
            scalarParams::get,
            (name, comp) -> {
                Translation2d v2 = (vec2Params != null) ? vec2Params.get(name) : null;
                if (v2 == null) return null;
                return (comp == 'X') ? v2.getX() : (comp == 'Y' ? v2.getY() : null);
            },
            (name, comp) -> {
                Translation3d v3 = (vec3Params != null) ? vec3Params.get(name) : null;
                if (v3 == null) return null;
                if (comp == 'X') return v3.getX();
                if (comp == 'Y') return v3.getY();
                if (comp == 'Z') return v3.getZ();
                return null;
            }
        );

        if (v == null) throw new RuntimeException("Expr eval failed: " + expr);
        return v.doubleValue();
    }

    private static void expandOffsetVars(
            ObjectNode root,
            java.util.Map<String, Double> params,
            java.util.Map<String, Translation2d> vec2Params) {

        if ((params == null || params.isEmpty()) && (vec2Params == null || vec2Params.isEmpty())) return;

        JsonNode fixtures = root.get("fixtures");
        if (fixtures == null || !fixtures.isArray()) return;

        for (JsonNode f : fixtures) {
            if (f != null && f.isObject()) {
                expandDerivedFromOffset((ObjectNode) f, params, vec2Params);
            }
        }
    }

    private static void expandDerivedFromOffset(
            ObjectNode fixtureOrDerivedFromOwner,
            java.util.Map<String, Double> params,
            java.util.Map<String, Translation2d> vec2Params) {

        JsonNode d = fixtureOrDerivedFromOwner.get("derivedFrom");
        if (d != null && d.isObject()) {
            ObjectNode dobj = (ObjectNode) d;

            JsonNode off = dobj.get("offset");
            if (off != null) {
                Double v = null;

                if (off.isNumber()) {
                    v = off.asDouble();
                } else if (off.isTextual()) {
                    String expr = off.asText();
                    if (expr != null) expr = expr.trim();
                    if (expr != null && !expr.isEmpty()) {
                        try {
                            v = evalExpr(expr, params, vec2Params);
                        } catch (RuntimeException ex) {
                            v = null;
                        }
                    }
                }

                if (v != null) dobj.put("offset", v.doubleValue());
            }

            expandDerivedFromOffset(dobj, params, vec2Params);
        }
    }

    public static void LoadIntoController(
            String filePath,
            AutoController controller,
            ModuleController moduleController,
            Supplier<Integer> startLocationProvider,
            boolean applyAllianceTransform)
            throws IOException {

        // CHANGED: split into preload + build to avoid Utility init cycles
        ParsedDefinitions defs = PreloadSeasonParams(filePath);
        LoadIntoControllerFromPreloaded(defs, controller, moduleController, startLocationProvider, applyAllianceTransform);
    }

    public static void LoadIntoController(
            String filePath,
            AutoController controller,
            ModuleController moduleController,
            Supplier<Integer> startLocationProvider)
            throws IOException {
        LoadIntoController(filePath, controller, moduleController, startLocationProvider, true);
    }

    private static String reqText(JsonNode root, String key) {
        JsonNode n = root.get(key);
        if (n == null || !n.isTextual()) throw new IllegalArgumentException("Missing/invalid string: " + key);
        return n.asText();
    }

    private static JsonNode reqArray(JsonNode root, String key) {
        JsonNode n = root.get(key);
        if (n == null || !n.isArray()) throw new IllegalArgumentException("Missing/invalid array: " + key);
        return n;
    }

    private static ArrayList<String> toStringList(JsonNode arr) {
        ArrayList<String> out = new ArrayList<>();
        for (JsonNode n : arr) out.add(n.asText());
        return out;
    }
}
