package frc.robot.misc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.action.Group;

public class Utility {
    static boolean initialized = false;

    public static Translation2d fieldSize = Translation2d.kZero;
    static Translation2d fieldCenter = new Translation2d(fieldSize.getX() / 2.0, fieldSize.getY() / 2.0);
    static Transform3d redStartTransform = null;

    static void Initialize() {
        if (!initialized) {
            if (fieldSize == Translation2d.kZero)
                throw new RuntimeException("Utility.fieldSize not set - you must set this to your Constants.fieldSize in RobotInit");
            
            fieldCenter = new Translation2d(fieldSize.getX() / 2.0, fieldSize.getY() / 2.0);
            redStartTransform = new Transform3d(new Translation3d(fieldCenter), new Rotation3d(new Rotation2d(Math.PI)));

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

        if (DriverStation.getLocation().isPresent())
          driverLocation = DriverStation.getLocation().getAsInt();

        return driverLocation;
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

    public static boolean isTravelGroup(Group group) {
        return group != Group.Score && group != Group.Approach && group != Group.Pickup;
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
