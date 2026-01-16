package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;

import com.fasterxml.jackson.databind.JsonNode;

/**
 * Raw, schema-shaped season definition loaded from auto.schema.json.
 * Intentionally does not bind to enums; trust JSON content.
 */
public final class AutoSeasonDefinition {
    public final String season;
    public final String version;
    public final String description;

    public final ArrayList<String> groups;
    public final ArrayList<String> locations;
    public final ArrayList<String> positions;
    public final ArrayList<String> actions;

    // Raw definitions (defer strong typing until needed)
    public final JsonNode poses;
    public final JsonNode fixtures;
    public final JsonNode events;
    public final JsonNode targets;
    public final JsonNode sequences;

    // resolved fixture graph (key: "type:index")
    public final HashMap<String, JsonNode> resolvedFixtures = new HashMap<>();

    public final JsonNode root;

    public AutoSeasonDefinition(
            JsonNode root,
            String season,
            String version,
            String description,
            ArrayList<String> groups,
            ArrayList<String> locations,
            ArrayList<String> positions,
            ArrayList<String> actions,
            JsonNode poses,
            JsonNode fixtures,
            JsonNode events,
            JsonNode targets,
            JsonNode sequences) {
        this.root = root;
        this.season = season;
        this.version = version;
        this.description = description;
        this.groups = groups;
        this.locations = locations;
        this.positions = positions;
        this.actions = actions;
        this.poses = poses;
        this.fixtures = fixtures;
        this.events = events;
        this.targets = targets;
        this.sequences = sequences;
    }
}
