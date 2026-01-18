package frc.robot.misc;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.action.ActionPose;

public class Utility {
    static boolean initialized = false;

    static Translation2d fieldSize = Translation2d.kZero;
    static Translation2d fieldCenter = new Translation2d(fieldSize.getX() / 2.0, fieldSize.getY() / 2.0);
    static Transform3d redStartTransform = null;

    // NEW: runtime-provided auto season info (set once after JSON load)
    private static ArrayList<String> travelGroups = new ArrayList<String>();

    // NEW: season params (numeric) for "$var" usage
    private static java.util.HashMap<String, Double> seasonParams = new java.util.HashMap<>();

    // CHANGED (semantic): vec2 params are stored in INCHES (to match scalar policy)
    private static HashMap<String, Translation2d> seasonVec2Params = new HashMap<>();

    // NEW: tool overrides (null = use DriverStation)
    static Alliance allianceOverride = null;
    static Integer driverLocationOverride = null;

    // --- NEW: computed field geometry cache (meters unless noted) ---
    private static boolean fieldConfigured = false;

    private static Translation2d fieldSizeM = Translation2d.kZero;
    private static Translation2d fieldCenterM = Translation2d.kZero;

    private static Translation2d startAreaM = Translation2d.kZero;

    private static Translation2d frameSizeM = Translation2d.kZero;
    private static Translation2d frameCenterM = Translation2d.kZero;
    private static Translation2d motorOffsetM = Translation2d.kZero;
    private static Translation2d motorPositionM = Translation2d.kZero;

    private static Translation2d robotSizeM = Translation2d.kZero;
    private static Translation2d robotCenterM = Translation2d.kZero;

    private static Translation2d blueStartPositionM = Translation2d.kZero;
    private static double[] blueStartYM = new double[] { 0.0, 0.0, 0.0 };

    private static double waypointOffsetM = 0.0;
    private static double alignOffsetM = 0.0;
    private static double scoreOffsetM = 0.0;

    public static void setRedStartTransform(Transform3d transform) {
        redStartTransform = transform;
    }

    public static void setFieldSize(Translation2d size) {
        fieldSize = size;
        fieldCenter = new Translation2d(fieldSize.getX() / 2.0, fieldSize.getY() / 2.0);
    }

    public static void SetTravelGroups(ArrayList<String> groups) {
        travelGroups = groups;
    }

    public static boolean isTravelGroup(String group) {
        if (group == null) return false;
        return travelGroups.contains(group.toLowerCase());
    }

    public static void SetSeasonParams(java.util.Map<String, Double> params) {
        seasonParams.clear();
        if (params != null) seasonParams.putAll(params);
    }

    public static double GetSeasonNumber(String key, double fallback) {
        if (key == null) return fallback;
        Double v = seasonParams.get(key);
        return (v != null) ? v.doubleValue() : fallback;
    }

    public static void SetSeasonVec2Params(java.util.Map<String, Translation2d> params) {
        seasonVec2Params.clear();
        if (params != null) seasonVec2Params.putAll(params);
    }

    public static Translation2d GetSeasonVec2(String key, Translation2d fallback) {
        if (key == null) return fallback;
        Translation2d v = seasonVec2Params.get(key);
        return (v != null) ? v : fallback;
    }

    public static Translation2d GetSeasonVec2Inches(String key, Translation2d fallbackInches) {
        return GetSeasonVec2(key, fallbackInches);
    }

    public static Translation2d GetSeasonVec2Meters(String key, Translation2d fallbackMeters) {
        Translation2d vIn = GetSeasonVec2(key, null);
        if (vIn == null) return fallbackMeters;
        return new Translation2d(inchesToMeters(vIn.getX()), inchesToMeters(vIn.getY()));
    }

    public static void setAllianceOverride(Alliance alliance) {
        allianceOverride = alliance;
    }

    public static void clearAllianceOverride() {
        allianceOverride = null;
    }

    public static void setDriverLocationOverride(Integer location) {
        driverLocationOverride = location;
    }

    public static void clearDriverLocationOverride() {
        driverLocationOverride = null;
    }

    static void Initialize() {
        if (!initialized) {
            System.err.println("Initializing Utility"); // CHANGED: was System.out.println

            if (fieldSize == Translation2d.kZero)
                throw new RuntimeException("Utility.fieldSize not set - you must set this to your Constants.fieldSize in RobotInit");

            initialized = true;
        }

    }

    // SwervePositions is an enum with an indexer value that allows matching up a motor module with the exact module state index it is assigned to
    public enum SwervePosition {
        LeftFront(0),
        RightFront(1),
        LeftRear(2),
        RightRear(3);

        private final int value;

        private SwervePosition(int value) {
            this.value = value;
        }
    
        public int getValue() {
            return value;
        }
    }

    public static boolean IsRedAlliance() {
        if (allianceOverride != null) return allianceOverride != Alliance.Blue;
        return DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() != Alliance.Blue;
    }

    public static int getDriverLocation() {
        if (driverLocationOverride != null) return driverLocationOverride.intValue();

        var driverLocation = 1;

        if (DriverStation.getLocation().isPresent()) {
          driverLocation = DriverStation.getLocation().getAsInt();
          System.err.printf("Found driver location %d from DS\n", driverLocation); // CHANGED
        }

        var myLocation = 0;
        try {
            myLocation = Integer.parseInt(SmartDashboard.getString("DB/String 6", "0"));
        } catch (Exception e) {
            
        }
        System.err.printf("Found driver location %d from dashboard\n", myLocation); // CHANGED

        var returnLocation = myLocation == 0 ? driverLocation : myLocation;
        System.err.printf("Returning driver location %d\n", returnLocation); // CHANGED

        return returnLocation;
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }

    public static double degreesToRadians(double degrees) {
        return degrees * 0.017453292519943295;
    }

    public static double radiansToDegrees(double radians) {
        return radians / 0.017453292519943295;
    }

    public static Rotation2d getLookat(Translation2d source, Translation2d target) {
        double dx = target.getX() - source.getX();
        double dy = target.getY() - source.getY();

        return Rotation2d.fromRadians(Math.atan2(dy,  dx));
    }

    public static double getMidpoint(double min, double max) {
        return (min + max) / 2.0;
    }

    public static Pose2d perpendicularBisectorAngle(Translation2d first, Translation2d second) {
        double x1 = first.getX();
        double y1 = first.getY();
        double x2 = second.getX();
        double y2 = second.getY();

        double dx = x2 - x1;
        double dy = y2 - y1;
        if (Math.abs(dx) < 1e-9 && Math.abs(dy) < 1e-9) {
            throw new IllegalArgumentException("Perpendicular bisector undefined for identical points");
        }
        double mx = (x1 + x2) / 2.0;
        double my = (y1 + y2) / 2.0;
    
        double theta = Math.atan2(dy, dx) + Math.PI / 2.0; // perpendicular to segment
        return new Pose2d(new Translation2d(mx, my), new Rotation2d(theta).plus(Rotation2d.k180deg));
    }

    // Project perpendicular to angle (left = +90°, right = -90°) in 2D
    public static Translation3d projectPerpendicular(Translation3d base, Rotation2d angle, double distance) {
        Rotation2d perp;

        perp = angle.plus(Rotation2d.fromDegrees(90));

        return base.plus(new Translation3d(new Translation2d(distance * perp.getCos(), distance * perp.getSin())));
    }

    // Project perpendicular to angle (left = +90°, right = -90°) in 2D
    public static Translation3d projectParallel(Translation3d base, Rotation2d angle, double distance) {
        return base.plus(new Translation3d(new Translation2d(distance * angle.getCos(), distance * angle.getSin())));
    }

    // Returns the intersection point (in meters) of two infinite lines defined by (p1, a1) and (p2, a2).
    // Optional.empty() is returned if lines are parallel or nearly parallel.
    public static Translation2d getIntersection(Pose2d point1, Pose2d point2) {
        var a1 = point1.getRotation();
        var a2 = point2.getRotation();
        var p1 = point1.getTranslation();
        var p2 = point2.getTranslation();

        final double dx1 = a1.getCos();
        final double dy1 = a1.getSin();
        final double dx2 = a2.getCos();
        final double dy2 = a2.getSin();

        // d1 x d2 (2D cross)
        final double denom = dx1 * dy2 - dy1 * dx2;
        if (Math.abs(denom) < 1e-9) {
            return point1.getTranslation();
        }

        final double rx = p2.getX() - p1.getX();
        final double ry = p2.getY() - p1.getY();

        // t along line1: (r x d2) / (d1 x d2)
        final double t = (rx * dy2 - ry * dx2) / denom;

        final double xi = p1.getX() + t * dx1;
        final double yi = p1.getY() + t * dy1;

        return new Translation2d(xi, yi);
    }  

    public static Translation3d transformToRedStart(Translation3d point) {
        if (!initialized)
            Initialize();

        return point.rotateAround(redStartTransform.getTranslation(), redStartTransform.getRotation());
    }
  
    public static Rotation2d rotateToRedStart(Rotation2d angle) {
        if (!initialized)
            Initialize();

        return angle.plus(redStartTransform.getRotation().toRotation2d());
    }

    public static Pose3d rotateToRedStart(Pose3d pose) {
        if (!initialized)
            Initialize();

        return pose.rotateAround(redStartTransform.getTranslation(), redStartTransform.getRotation());
    }

    public static Translation3d transformToAllianceStart(Translation3d point) {
        if (!initialized)
            Initialize();

        if (IsRedAlliance())
            return transformToRedStart(point);
        else
            return point;
    }

    public static Rotation2d rotateToAllianceStart(Rotation2d angle) {
        if (!initialized)
            Initialize();

        if (IsRedAlliance())
            return rotateToRedStart(angle);
        else
            return angle;
    }

    public static Pose3d rotateToAllianceStart(Pose3d pose) {
        if (!initialized)
            Initialize();

        if (IsRedAlliance())
            return rotateToRedStart(pose);
        else
            return pose;
    }

    public static ActionPose GetActionPose(String group, String location, int locationIndex, String position, String action, String moduleID, ArrayList<ActionPose> actionPoses) {
        for (ActionPose pose : actionPoses) {
            if (
                (pose.group.equalsIgnoreCase(group) || "any".equalsIgnoreCase(pose.group))
                && (pose.locationIndex == locationIndex || pose.locationIndex == -1)
                && (pose.location.equalsIgnoreCase(location) || "any".equalsIgnoreCase(pose.location))
                && (pose.position.equalsIgnoreCase(position) || "any".equalsIgnoreCase(pose.position))
                && (pose.action.equalsIgnoreCase(action) || "any".equalsIgnoreCase(pose.action))
            ) {
                System.out.printf("%s GetActionPose: Matched %s %s %d %s %s\n", moduleID, pose.group, pose.location, pose.locationIndex, pose.position, pose.action);
                return pose;
            }
        }

        return null;
    }

    public static void ConfigureFieldFromSeasonParams() {
        // Idempotent; safe to call every mode init after JSON load
        // (but will re-evaluate if season params change at runtime)
        // If you truly want "only once", gate on fieldConfigured.
        // We'll allow recompute to avoid "stale params" bugs.
        fieldConfigured = true;

        // Field dimensions are in METERS in FRC docs typically; keep as meters here.
        // JSON numeric params policy: scalars/vec2 are stored in INCHES elsewhere,
        // but fieldSize is better as a vec2 meters OR explicit meters scalars.
        // Support both: vec2 in inches ("fieldSize") OR meters scalars ("fieldSizeXM", "fieldSizeYM").
        Translation2d fieldSizeIn = GetSeasonVec2Inches("fieldSize", null);
        if (fieldSizeIn != null) {
            fieldSizeM = new Translation2d(inchesToMeters(fieldSizeIn.getX()), inchesToMeters(fieldSizeIn.getY()));
        } else {
            double xM = GetSeasonNumber("fieldSizeXM", 17.55);
            double yM = GetSeasonNumber("fieldSizeYM", 8.05);
            fieldSizeM = new Translation2d(xM, yM);
        }
        fieldCenterM = new Translation2d(fieldSizeM.getX() / 2.0, fieldSizeM.getY() / 2.0);

        // startArea: fallback matches your old comments
        double startAreaXM = GetSeasonNumber("startAreaXM", (fieldSizeM.getX() / 2.0) - 7.56);
        double startAreaYM = GetSeasonNumber("startAreaYM", 3.72);
        startAreaM = new Translation2d(startAreaXM, startAreaYM);

        // Robot frame (inches in JSON)
        Translation2d frameSizeIn = GetSeasonVec2Inches("frameSize", new Translation2d(27.5, 32.375));
        frameSizeM = new Translation2d(inchesToMeters(frameSizeIn.getX()), inchesToMeters(frameSizeIn.getY()));
        frameCenterM = new Translation2d(frameSizeM.getX() / 2.0, frameSizeM.getY() / 2.0);

        Translation2d motorOffsetIn = GetSeasonVec2Inches("motorOffset", new Translation2d(4.0, 4.5));
        motorOffsetM = new Translation2d(inchesToMeters(motorOffsetIn.getX()), inchesToMeters(motorOffsetIn.getY()));
        motorPositionM = frameCenterM.minus(motorOffsetM);

        double bumperWidthIn = GetSeasonNumber("bumperWidth", 4.0);
        double bumperWidthM = inchesToMeters(bumperWidthIn);
        robotSizeM = frameSizeM.plus(new Translation2d(bumperWidthM, bumperWidthM));
        robotCenterM = new Translation2d(robotSizeM.getX() / 2.0, robotSizeM.getY() / 2.0);

        double startPaddingM = (startAreaM.getX() / 2.0) - robotCenterM.getX();
        Translation2d robotOffsetM = robotCenterM.plus(new Translation2d(startPaddingM, startPaddingM * 2.0));

        blueStartPositionM = new Translation2d(fieldCenterM.getX() - startAreaM.getX() + robotOffsetM.getX(), fieldCenterM.getY());

        // blueStartY (inches list) fallback from your comments
        blueStartYM = new double[] {
            inchesToMeters(GetSeasonNumber("blueStartY1", 127.38)),
            inchesToMeters(GetSeasonNumber("blueStartY2", 84.03)),
            inchesToMeters(GetSeasonNumber("blueStartY3", 36.458)),
        };

        // Misc pathing offsets
        waypointOffsetM = GetSeasonNumber("waypointOffsetM", robotSizeM.getNorm() * 1.25);
        alignOffsetM = GetSeasonNumber("alignOffsetM", robotSizeM.getNorm() * 0.75);
        scoreOffsetM = inchesToMeters(GetSeasonNumber("scoreOffset", -3.0));
    }

    private static void ensureFieldConfigured() {
        if (!fieldConfigured) {
            // Allows simulation/unit tests not going through Robot.commonInit()
            ConfigureFieldFromSeasonParams();
        }
    }

    public static Pose3d getMyStartPose() {
        var myLocation = getDriverLocation();
        return getStartPose(myLocation);
    }
    
    public static Pose3d getStartPose(int index) {
        if (index < 1) index = 1;

        // NEW: pull computed geometry from Utility (JSON-backed)
        Translation2d blueStartPosition = GetBlueStartPositionMeters();
        double[] blueStartY = GetBlueStartYMeters();

        // Clamp index to available entries to avoid OOB
        if (index > blueStartY.length) index = blueStartY.length;

        var thisStartPosition = blueStartPosition.plus(new Translation2d(0.0, blueStartY[index - 1]));
        var transformedPosition = transformToAllianceStart(new Translation3d(thisStartPosition));
        return new Pose3d(transformedPosition, Rotation3d.kZero);
    }

    // --- NEW: getters used by other code (meters) ---
    public static Translation2d GetMotorPositionMeters() { ensureFieldConfigured(); return motorPositionM; }
    public static Translation2d GetBlueStartPositionMeters() { ensureFieldConfigured(); return blueStartPositionM; }
    public static double[] GetBlueStartYMeters() { ensureFieldConfigured(); return blueStartYM; }

    // Optional (in case other code needs them later)
    public static Translation2d GetFieldSizeMeters() { ensureFieldConfigured(); return fieldSizeM; }
    public static Translation2d GetFieldCenterMeters() { ensureFieldConfigured(); return fieldCenterM; }
    public static double GetWaypointOffsetMeters() { ensureFieldConfigured(); return waypointOffsetM; }
    public static double GetAlignOffsetMeters() { ensureFieldConfigured(); return alignOffsetM; }
    public static double GetScoreOffsetMeters() { ensureFieldConfigured(); return scoreOffsetM; }
}
