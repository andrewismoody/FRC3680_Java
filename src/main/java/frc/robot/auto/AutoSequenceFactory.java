package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.JsonNode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.action.ActionPose;
import frc.robot.misc.Utility;
import frc.robot.modules.ModuleState;
import frc.robot.modules.RobotModule;

public final class AutoSequenceFactory {
    private final AutoController controller;

    private final HashMap<String, JsonNode> eventByName = new HashMap<>();
    private final HashMap<String, JsonNode> poseByName = new HashMap<>();

    // NEW: for fixture resolution by (type:index)
    private final AutoSeasonDefinition def;

    private final Supplier<Integer> startLocationProvider;

    // Start position selection:
    // 0 = ignore startN, 1..3 = explicit, -1 = derive via startPoseProvider
    private int startPosition = 0;

    public AutoSequenceFactory(AutoController controller, AutoSeasonDefinition def, Supplier<Integer> startLocationProvider) {
        this.controller = controller;
        this.def = def;
        this.startLocationProvider = startLocationProvider;

        // Default behavior: derive start location from provider
        this.startPosition = -1;

        for (JsonNode e : def.events) {
            eventByName.put(e.get("name").asText(), e);
        }

        for (JsonNode p : def.poses) {
            poseByName.put(p.get("name").asText(), p);
        }

        for (JsonNode t : def.targets) {
            String moduleID = t.get("module").asText();
            AutoTarget at = buildAutoTarget(t);
            
            if (at != null) {
                ActionPose ap = buildActionPoseForTarget(t, at);
                for (var module : controller.GetModuleController().GetAllModules()) {
                    if (module.GetModuleID().equalsIgnoreCase(moduleID))
                        module.AddActionPose(ap);
                }

                var driveModule = controller.GetModuleController().GetDriveModule();
                if (driveModule.GetModuleID().equalsIgnoreCase(moduleID))
                    driveModule.AddActionPose(ap);
            } else {
                System.out.printf("AutoSequenceFactory: failed to build target for module '%s'\n", moduleID);
            }
        }
    }

    /** 0 ignores startN arrays. 1..3 prepends startN, -1 attempts to derive from current pose. */
    public AutoSequenceFactory setStartPosition(int pos) {
        this.startPosition = pos;
        return this;
    }

    public AutoSequence fromSequenceJson(JsonNode seqObj) {
        String name = seqObj.get("name").asText();
        AutoSequence seq = new AutoSequence(name, controller);

        ArrayList<String> orderedEventNames = new ArrayList<>();

        // TODO: this needs to be resolved at the right time, currently it evaluates before the driver station is even connected
        int resolvedStart = resolveStartPosition();
        if (resolvedStart == 1 && seqObj.has("start1")) orderedEventNames.addAll(toStringList(seqObj.get("start1")));
        if (resolvedStart == 2 && seqObj.has("start2")) orderedEventNames.addAll(toStringList(seqObj.get("start2")));
        if (resolvedStart == 3 && seqObj.has("start3")) orderedEventNames.addAll(toStringList(seqObj.get("start3")));

        orderedEventNames.addAll(toStringList(seqObj.get("events")));

        boolean first = true;
        for (String eventName : orderedEventNames) {
            AutoEvent evt = fromEventName(eventName);
            if (evt == null) {
                System.out.printf("AutoSequenceFactory: sequence '%s' references unknown/invalid event '%s'\n", name, eventName);
                continue;
            }
            if (first) {
                seq.BeginWith(evt);
                first = false;
            } else {
                seq.Then(evt);
            }
        }

        return seq;
    }

    private AutoEvent fromEventName(String name) {
        JsonNode e = eventByName.get(name);
        if (e == null) return null;

        String type = e.get("type").asText();
        boolean parallel = e.get("parallel").asBoolean();

        switch (type) {
            case "await": {
                String poseName = e.get("pose").asText();
                
                return new AutoEventTarget(
                        name,
                        parallel,
                        buildActionPoseFromPoseName(poseName),
                        AutoEvent.EventType.AwaitTarget,
                        controller);
            }

            case "time": {
                int ms = e.get("milliseconds").asInt();
                AutoEventTime timeEvt = new AutoEventTime(name, parallel, ms, AutoEvent.EventType.Void, controller);

                // If a module+value is specified, treat it as an "apply" action that runs once at event start.
                // This is distinct from "targets" (AutoTarget), which should be handled via target definitions.
                String triggerType = e.hasNonNull("triggerType") ? e.get("triggerType").asText() : null;
                if (triggerType != null && !"none".equalsIgnoreCase(triggerType)) {
                    String moduleName = e.get("triggerModule").asText();
                    boolean invert = e.has("triggerInvert") && e.get("triggerInvert").asBoolean();

                    RobotModule rm = findModule(moduleName);
                    if (rm == null) {
                        System.out.printf("AutoSequenceFactory: event '%s' triggerModule '%s' not found\n", name, moduleName);
                        return timeEvt;
                    }


                    if ("boolean".equalsIgnoreCase(triggerType)) {
                        boolean v = e.get("triggerValue").asBoolean();
                        if (invert) timeEvt.SetBoolEvent(v, rm::ApplyInverse); else timeEvt.SetBoolEvent(v, rm::ApplyValue);
                    }
                }

                return timeEvt;
            }

            default:
                System.out.printf("AutoSequenceFactory: unknown event type '%s' for event '%s'\n", type, name);
                return null;
        }
    }

    private ActionPose buildActionPoseFromPoseName(String poseName) {
        JsonNode p = poseByName.get(poseName);
        if (p == null) return null;

        String group = p.get("group").asText();
        String location = p.get("location").asText();
        int index = p.get("index").asInt();
        String position = p.get("position").asText();
        String action = p.get("action").asText();

        // IMPORTANT: loose association model:
        // - Do NOT bind a target here.
        // - Each module resolves its own target when the ActionPose is dispatched.
        return new ActionPose(group, location, index, position, action, null);
    }

    private ActionPose buildActionPoseForTarget(JsonNode p, AutoTarget at) {
        String group = p.hasNonNull("group") ? p.get("group").asText() : "any";
        String location = p.hasNonNull("location") ? p.get("location").asText() : "any";
        int index = p.hasNonNull("index") ? p.get("index").asInt() : -1;
        String position = p.hasNonNull("position") ? p.get("position").asText() : "any";
        String action = p.hasNonNull("action") ? p.get("action").asText() : "any";

        return new ActionPose(group, location, index, position, action, at);
    }

    private AutoTarget buildAutoTarget(JsonNode t) {
        if (t == null) return null;

        // target.schema.json supports oneOf: measurement | state | translation | fixture
        if (t.hasNonNull("measurement")) {
            return new AutoTarget(t.get("measurement").asDouble());
        }

        if (t.hasNonNull("state")) {
            String s = t.get("state").asText();
            try {
                return new AutoTarget(ModuleState.valueOf(s));
            } catch (IllegalArgumentException ex) {
                System.out.printf("AutoSequenceFactory: invalid ModuleState '%s' for target\n", s);
                return null;
            }
        }

        if (t.hasNonNull("translation")) {
            var myTranslation = t.get("translation");

            Pose3d tr = parseTranslation(t.get("translation"));
            if (tr == null) return null;

            var myPosition = myTranslation.get("position");
            var myRotation = myTranslation.get("rotation");

            if (myPosition != null && myRotation != null) return new AutoTarget(tr.getTranslation(), tr.getRotation().toRotation2d());
            if (myPosition != null) return new AutoTarget(tr.getTranslation(), false);
            if (myRotation != null) return new AutoTarget(tr.getRotation().toRotation2d());

            return null;
        }

        if (t.hasNonNull("fixture")) {
            JsonNode fref = t.get("fixture");
            String ftype = fref.path("type").asText("");
            int findex = fref.path("index").asInt(-1);
            String key = ftype + ":" + findex;

            JsonNode resolved = def.resolvedFixtures.get(key);
            if (resolved == null) {
                System.out.printf("AutoSequenceFactory: unresolved fixture ref '%s' for target\n", key);
                return null;
            }

            // Minimal assumption: fixture contains a translation usable as a target position
            if (resolved.hasNonNull("translation")) {
                var myTranslation = resolved.get("translation");

                Pose3d tr = parseTranslation(myTranslation);
                if (tr == null) return null;

                var myPosition = myTranslation.get("position");
                var myRotation = myTranslation.get("rotation");

                if (myPosition != null && myRotation != null) return new AutoTarget(tr.getTranslation(), tr.getRotation().toRotation2d());
                if (myPosition != null) return new AutoTarget(tr.getTranslation(), false);
                if (myRotation != null) return new AutoTarget(tr.getRotation().toRotation2d());
            }
            return null;
        }

        return null;
    }

    // NEW: translation.schema.json parser (position + rotation with units)
    private static Pose3d parseTranslation(JsonNode t) {
        if (t == null || t.isNull()) return null;

        Translation3d posOut = new Translation3d();
        Rotation2d rotOut = new Rotation2d();

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

        return new Pose3d(posOut, new Rotation3d(rotOut));
    }

    private int resolveStartPosition() {
        if (startPosition == 1 || startPosition == 2 || startPosition == 3) return startPosition;
        if (startPosition != -1) return 0;

        return startLocationProvider.get();
    }

    private RobotModule findModule(String name) {
        try {
            return controller.GetModuleController().GetModule(name);
        } catch (Throwable t) {
            return null;
        }
    }

    private static ArrayList<String> toStringList(JsonNode arr) {
        ArrayList<String> out = new ArrayList<>();
        for (JsonNode n : arr) out.add(n.asText());
        return out;
    }
}
