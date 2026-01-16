package frc.robot.misc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.action.Group;

public class Utility {
    static boolean initialized = false;

    static Translation2d fieldSize = Translation2d.kZero;
    static Translation2d fieldCenter = new Translation2d(fieldSize.getX() / 2.0, fieldSize.getY() / 2.0);
    static Transform3d redStartTransform = null;

    public static void setRedStartTransform(Transform3d transform) {
        redStartTransform = transform;
    }

    public static void setFieldSize(Translation2d size) {
        fieldSize = size;
        fieldCenter = new Translation2d(fieldSize.getX() / 2.0, fieldSize.getY() / 2.0);
    }

    static void Initialize() {
        if (!initialized) {
            System.out.println("Initializing Utility");

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
        return DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() != Alliance.Blue;
    }

    public static int getDriverLocation() {
        var driverLocation = 1;

        if (DriverStation.getLocation().isPresent()) {
          driverLocation = DriverStation.getLocation().getAsInt();
          System.out.printf("Found driver location %d from DS\n", driverLocation);
        }

        var myLocation = 0;
        try {
            myLocation = Integer.parseInt(SmartDashboard.getString("DB/String 6", "0"));
        } catch (Exception e) {
            
        }
        System.out.printf("Found driver location %d from dashboard\n", myLocation);

        var returnLocation = myLocation == 0 ? driverLocation : myLocation;
        System.out.printf("Returning driver location %d\n", returnLocation);

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

    // COPILOT: move this to autocontroller
    public static boolean isTravelGroup(String group) {
        return group != Group.Score && group != Group.ApproachLeft && group != Group.ApproachRight && group != Group.Pickup;
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
}
