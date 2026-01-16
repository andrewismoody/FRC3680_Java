package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import frc.robot.modules.ModuleController;
import frc.robot.modules.RobotModule;
import frc.robot.z2025.Robot;

/**
 * Runs post-load validation of a season auto definition.
 *
 * Goals:
 * - Catch cross-reference errors that JSON Schema can't (missing names, unresolved fixtures, missing modules).
 * - "Play through" each sequence by creating the same runtime objects AutoSequenceFactory will create.
 *
 * Non-goals:
 * - No timing simulation
 * - No hardware actuation (does not call module ApplyValue/ApplyInverse)
 */
public final class AutoValidator {
    private AutoValidator() {}

    public static final class Result {
        public final List<String> errors = new ArrayList<>();
        public final List<String> warnings = new ArrayList<>();
        public final List<String> info = new ArrayList<>();

        public boolean ok() { return errors.isEmpty(); }

        public void printToStdout() {
            for (String i : info) System.out.printf("AUTO-VALIDATOR INFO: %s\n", i);
            for (String w : warnings) System.out.printf("AUTO-VALIDATOR WARNING: %s\n", w);
            for (String e : errors) System.out.printf("AUTO-VALIDATOR ERROR: %s\n", e);
            System.out.printf("AUTO-VALIDATOR: %d error(s), %d warning(s)\n", errors.size(), warnings.size());
        }
    }

    public static Result Validate(
            AutoController controller,
            AutoSeasonDefinition def,
            ModuleController modules,
            Supplier<Integer> startLocationProvider) {

        Result r = new Result();
        r.info.add("Light Validation");
        if (controller == null) {
            r.errors.add("controller is null");
            return r;
        }
        if (def == null) {
            r.errors.add("season definition is null");
            return r;
        }
        if (modules == null) {
            r.errors.add("modules is null");
            return r;
        }

        // Index poses/events by name for cross-reference validation
        Set<String> poseNames = new HashSet<>();
        for (JsonNode p : def.poses) {
            String n = p.path("name").asText("");
            if (n.isBlank()) r.errors.add("pose missing non-empty 'name'");
            else if (!poseNames.add(n)) r.errors.add("duplicate pose name: " + n);
        }

        Set<String> eventNames = new HashSet<>();
        for (JsonNode e : def.events) {
            String n = e.path("name").asText("");
            if (n.isBlank()) r.errors.add("event missing non-empty 'name'");
            else if (!eventNames.add(n)) r.errors.add("duplicate event name: " + n);

            String type = e.path("type").asText("");
            if (!"await".equals(type) && !"time".equals(type)) {
                r.errors.add("event '" + n + "' has invalid type '" + type + "'");
                continue;
            }

            if ("await".equals(type)) {
                String pose = e.path("pose").asText("");
                if (pose.isBlank()) r.errors.add("event '" + n + "' type=await missing 'pose'");
                else if (!poseNames.contains(pose)) r.errors.add("event '" + n + "' references missing pose '" + pose + "'");
            } else {
                if (!e.hasNonNull("milliseconds")) r.errors.add("event '" + n + "' type=time missing 'milliseconds'");
                String triggerType = e.hasNonNull("triggerType") ? e.get("triggerType").asText("") : "";
                if ("boolean".equalsIgnoreCase(triggerType)) {
                    String triggerModule = e.path("triggerModule").asText("");
                    if (triggerModule.isBlank()) r.errors.add("event '" + n + "' triggerType=boolean missing 'triggerModule'");
                    else {
                        RobotModule rm = safeGetModule(modules, triggerModule);
                        if (rm == null) r.errors.add("event '" + n + "' triggerModule '" + triggerModule + "' not found");
                    }
                    if (!e.hasNonNull("triggerValue")) r.errors.add("event '" + n + "' triggerType=boolean missing 'triggerValue'");
                }
            }
        }

        // REPLACE: "awaited pose must have target" with "there exists at least one target matching the pose tuple"
        String driveId = null;
        try { driveId = modules.GetDriveModule().GetModuleID(); } catch (Throwable t) {}
        if (driveId == null) driveId = "swerveDrive";

        for (JsonNode e : def.events) {
            if (!"await".equals(e.path("type").asText(""))) continue;

            String poseName = e.path("pose").asText("");
            JsonNode pose = null;
            for (JsonNode p : def.poses) {
                if (poseName.equals(p.path("name").asText(""))) { pose = p; break; }
            }
            if (pose == null) continue;

            // Loose association: pose doesn't embed target. Validate that a matching target exists in def.targets.
            if (!hasTargetForPose(def, pose, driveId)) {
                r.warnings.add("await event '" + e.path("name").asText("") + "' pose '" + poseName
                        + "' has no matching target entry for drive module '" + driveId + "'; event may no-op");
            }
        }

        // Fixtures: validate resolved fixtures graph is complete for every referenced fixture in targets
        // (We only check that resolvedFixtures contains the requested key and it has a translation when used for a drive-ish target.)
        for (JsonNode t : def.targets) {
            validateTarget(def, t, r, modules);
        }

        // "Play through": instantiate sequences through the factory and force event creation for all referenced names.
        AutoSequenceFactory factory = new AutoSequenceFactory(controller, def, startLocationProvider);

        for (JsonNode s : def.sequences) {
            String seqName = s.path("name").asText("");
            if (seqName.isBlank()) {
                r.errors.add("sequence missing non-empty 'name'");
                continue;
            }

            // Validate event name references exist
            if (!s.hasNonNull("events") || !s.get("events").isArray()) {
                r.errors.add("sequence '" + seqName + "' missing array 'events'");
                continue;
            }

            for (JsonNode evNameNode : s.get("events")) {
                String evName = evNameNode.asText("");
                if (!eventNames.contains(evName)) r.errors.add("sequence '" + seqName + "' references missing event '" + evName + "'");
            }

            // Force factory instantiation; this exercises buildActionPoseFromPoseName + buildAutoTarget
            try {
                AutoSequence built = factory.fromSequenceJson(s);
                if (built == null) {
                    r.errors.add("sequence '" + seqName + "': factory returned null");
                }
            } catch (Throwable ex) {
                r.errors.add("sequence '" + seqName + "': exception during build: " + ex.getMessage());
            }
        }

        return r;
    }

    // NEW: tuple-match helper for the loose association model
    private static boolean hasTargetForPose(AutoSeasonDefinition def, JsonNode pose, String moduleId) {
        if (def == null || def.targets == null || pose == null) return false;

        String g = pose.path("group").asText("");
        String loc = pose.path("location").asText("");
        int idx = pose.path("index").asInt(-999);

        for (JsonNode t : def.targets) {
            if (t == null || t.isNull()) continue;
            if (!moduleId.equalsIgnoreCase(t.path("module").asText(""))) continue;
            if (!g.equalsIgnoreCase(t.path("group").asText(""))) continue;
            if (!loc.equalsIgnoreCase(t.path("location").asText(""))) continue;
            if (idx != t.path("index").asInt(-999)) continue;
            return true;
        }
        return false;
    }

    public static Result ValidateFull(
            AutoController controller,
            AutoSeasonDefinition def,
            ModuleController modules,
            Supplier<Integer> startLocationProvider) {

        Result r = Validate(controller, def, modules, startLocationProvider);
        if (def == null) {
            System.out.println("Season definition is null; skipping full validation");
            return r;
        }
        if (Robot.isReal()) {
            System.out.println("Robot is Real; skipping full validation");
            return r; // skip full validation on real robot to avoid timing issues
        }

        r.info.add("Full Validation");
        AutoSequenceFactory factory = new AutoSequenceFactory(controller, def, startLocationProvider);

        // FIX: build tiny sequences for each event (loop/braces were broken)
        for (JsonNode e : def.events) {
            String eventName = e.path("name").asText("");
            try {
                ObjectNode tmpSeq = new ObjectMapper().createObjectNode();
                tmpSeq.put("name", "__validate_event__" + eventName);
                ArrayNode arr = tmpSeq.arrayNode();
                arr.add(eventName);
                tmpSeq.set("events", arr);

                AutoSequence seq = factory.fromSequenceJson(tmpSeq);
                if (seq == null) r.errors.add("full: failed to build event '" + eventName + "' into a sequence");
            } catch (Throwable ex) {
                r.errors.add("full: exception while building event '" + eventName + "': " + ex.getMessage());
            }
        }

        // 3) Force fixture materialization audit: anything in resolvedFixtures should have schema-normalized translation if it provided/derived one
        for (var entry : def.resolvedFixtures.entrySet()) {
            String key = entry.getKey();
            JsonNode f = entry.getValue();
            if (f == null) {
                r.errors.add("full: resolved fixture '" + key + "' is null");
                continue;
            }
            if (f.has("translation") && !f.get("translation").isObject()) {
                r.errors.add("full: resolved fixture '" + key + "' has non-object translation");
            }
        }

        return r;
    }

    private static void validateTarget(AutoSeasonDefinition def, JsonNode t, Result r, ModuleController modules) {
        if (t == null || t.isNull()) {
            r.errors.add("null target entry");
            return;
        }

        String module = t.path("module").asText("");
        if (module.isBlank()) {
            r.errors.add("target missing 'module'");
            return;
        }

        // FIX: drive module may not be in the generic module registry; accept DriveModule ID too.
        if (modules != null && !moduleExists(modules, module)) {
            r.errors.add("target references missing module '" + module + "'");
        }

        boolean hasMeasurement = t.hasNonNull("measurement");
        boolean hasState = t.hasNonNull("state");
        boolean hasTranslation = t.hasNonNull("translation");
        boolean hasFixture = t.hasNonNull("fixture");

        int count = (hasMeasurement ? 1 : 0) + (hasState ? 1 : 0) + (hasTranslation ? 1 : 0) + (hasFixture ? 1 : 0);
        if (count == 0) r.errors.add("target(module=" + module + ") missing oneOf: measurement/state/translation/fixture");
        if (count > 1) r.errors.add("target(module=" + module + ") has multiple oneOf: measurement/state/translation/fixture");

        if (hasFixture) {
            JsonNode fref = t.get("fixture");
            String ftype = fref.path("type").asText("");
            int findex = fref.path("index").asInt(-1);
            if (ftype.isBlank() || findex < 0) {
                r.errors.add("target(module=" + module + ") has invalid fixture ref: type='" + ftype + "' index=" + findex);
                return;
            }
            String key = ftype + ":" + findex;
            JsonNode resolved = def.resolvedFixtures.get(key);
            if (resolved == null) {
                r.errors.add("target(module=" + module + ") fixture ref '" + key + "' not resolvable");
                return;
            }
            if (!resolved.hasNonNull("translation")) {
                r.warnings.add("target(module=" + module + ") fixture '" + key + "' resolved but has no translation (may be ok if non-positional target)");
            }
        }
    }

    private static boolean moduleExists(ModuleController modules, String name) {
        if (name == null || name.isBlank()) return false;

        try {
            var drive = modules.GetDriveModule();
            if (drive != null && name.equalsIgnoreCase(drive.GetModuleID())) return true;
        } catch (Throwable t) {
            // ignore and fall through
        }

        return safeGetModule(modules, name) != null;
    }

    private static RobotModule safeGetModule(ModuleController modules, String name) {
        try {
            return modules.GetModule(name);
        } catch (Throwable t) {
            return null;
        }
    }
}
