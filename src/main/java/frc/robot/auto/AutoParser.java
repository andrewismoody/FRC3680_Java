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

    // CHANGED: definitions-only result now includes a ready-to-use AutoSeasonDefinition
    public static final class ParsedDefinitions {
        public final AutoSeasonDefinition def;

        // optional extras that are not part of AutoSeasonDefinition ctor wiring
        public final JsonNode root; // keep if callers still want raw access

        private ParsedDefinitions(AutoSeasonDefinition def, JsonNode root) {
            this.def = def;
            this.root = root;
        }
    }

    // CHANGED: now returns ParsedDefinitions(def=AutoSeasonDefinition(...)) with params/travelGroups applied to def
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

        HashMap<String, Double> params = new HashMap<>();
        if (root.hasNonNull("params") && root.get("params").isObject()) {
            var it = root.get("params").fields();
            while (it.hasNext()) {
                var e = it.next();
                if (e.getValue().isNumber()) params.put(e.getKey(), e.getValue().asDouble());
            }
        }

        // keep behavior consistent with LoadIntoController
        if (root.isObject()) {
            expandOffsetVars((ObjectNode) root, params);
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

    public static void LoadIntoController(
            String filePath,
            AutoController controller,
            ModuleController moduleController,
            Supplier<Integer> startLocationProvider,
            boolean applyAllianceTransform) // NEW
            throws IOException {

        ParsedDefinitions defs = LoadDefinitions(filePath);
        AutoSeasonDefinition def = defs.def;

        controller.SetDefinitions(def.groups, def.locations, def.positions, def.actions);

        Utility.SetSeasonParams(def.params);

        FixtureResolver.ResolveAll(def, applyAllianceTransform); // CHANGED

        controller.SetSeasonDefinition(def);

        AutoSequenceFactory factory = new AutoSequenceFactory(controller, def, startLocationProvider);
        for (JsonNode seqNode : def.sequences) {
            AutoSequence seq = factory.fromSequenceJson(seqNode);
            if (seq != null) controller.AddSequence(seq);
        }
    }

    // Keep old signature for existing callers (defaults to prior behavior = true)
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

    // NEW: expand derivedFrom.offset = "$name" into numeric value from params
    private static void expandOffsetVars(ObjectNode root, java.util.Map<String, Double> params) {
        if (params == null || params.isEmpty()) return;

        // Only touch fixtures[].derivedFrom recursively (keeps this minimal/safe)
        JsonNode fixtures = root.get("fixtures");
        if (fixtures == null || !fixtures.isArray()) return;

        for (JsonNode f : fixtures) {
            if (f != null && f.isObject()) {
                expandDerivedFromOffset((ObjectNode) f, params);
            }
        }
    }

    private static void expandDerivedFromOffset(ObjectNode fixtureOrDerivedFromOwner, java.util.Map<String, Double> params) {
        JsonNode d = fixtureOrDerivedFromOwner.get("derivedFrom");
        if (d != null && d.isObject()) {
            ObjectNode dobj = (ObjectNode) d;

            JsonNode off = dobj.get("offset");
            if (off != null && off.isTextual()) {
                String s = off.asText();
                if (s != null) s = s.trim();

                double sign = 1.0;
                if (s != null && s.startsWith("-")) {
                    sign = -1.0;
                    s = s.substring(1).trim();
                }

                if (s != null && s.startsWith("$")) {
                    String key = s.substring(1);
                    Double v = params.get(key);
                    if (v != null) dobj.put("offset", sign * v.doubleValue());
                }
            }

            // recurse for nested derivedFrom chains
            expandDerivedFromOffset(dobj, params);
        }
    }
}
