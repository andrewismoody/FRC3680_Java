package frc.robot.auto;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
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

    public static void LoadIntoController(
            String filePath,
            AutoController controller,
            ModuleController moduleController,
            Supplier<Integer> startLocationProvider)
            throws IOException {

        JsonNode root = MAPPER.readTree(Files.readString(Path.of(filePath)));

        String season = reqText(root, "season");
        String version = reqText(root, "version");
        String description = reqText(root, "description");

        ArrayList<String> groups = toStringList(reqArray(root, "groups"));
        ArrayList<String> locations = toStringList(reqArray(root, "locations"));
        ArrayList<String> positions = toStringList(reqArray(root, "positions"));
        ArrayList<String> actions = toStringList(reqArray(root, "actions"));

        // NEW: optional travelGroups
        ArrayList<String> travelGroups = root.has("travelGroups") && root.get("travelGroups").isArray()
                ? toStringList(root.get("travelGroups"))
                : new ArrayList<>();

        controller.SetDefinitions(groups, locations, positions, actions);

        JsonNode poses = reqArray(root, "poses");
        JsonNode fixtures = reqArray(root, "fixtures");
        JsonNode events = reqArray(root, "events");
        JsonNode targets = reqArray(root, "targets");
        JsonNode sequences = reqArray(root, "sequences");

        AutoSeasonDefinition def = new AutoSeasonDefinition(
                root, season, version, description,
                groups, locations, positions, actions,
                poses, fixtures, events, targets, sequences);

        // NEW: stash travelGroups on the definition (add field to AutoSeasonDefinition)
        def.travelGroups = travelGroups;

        // NEW: parse optional params map
        if (root.hasNonNull("params") && root.get("params").isObject()) {
            var it = root.get("params").fields();
            while (it.hasNext()) {
                var e = it.next();
                if (e.getValue().isNumber()) def.params.put(e.getKey(), e.getValue().asDouble());
            }
        }

        // NEW: expand "$var" references in-place (currently: derivedFrom.offset)
        if (root.isObject()) {
            expandOffsetVars((ObjectNode) root, def.params);
        }

        // NEW: publish params to Utility for runtime access (like travelGroups)
        Utility.SetSeasonParams(def.params);

        FixtureResolver.ResolveAll(def);

        controller.SetSeasonDefinition(def);

        AutoSequenceFactory factory = new AutoSequenceFactory(controller, def, startLocationProvider);
        for (JsonNode seqNode : sequences) {
            AutoSequence seq = factory.fromSequenceJson(seqNode);
            if (seq != null) controller.AddSequence(seq);
        }
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
                if (s.startsWith("$")) {
                    String key = s.substring(1);
                    Double v = params.get(key);
                    if (v != null) dobj.put("offset", v.doubleValue());
                }
            }

            // recurse for nested derivedFrom chains
            expandDerivedFromOffset(dobj, params);
        }
    }
}
