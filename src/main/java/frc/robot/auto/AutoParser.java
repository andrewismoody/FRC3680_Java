package frc.robot.auto;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

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
}
