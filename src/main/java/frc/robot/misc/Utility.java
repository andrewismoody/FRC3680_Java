package frc.robot.misc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utility {
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

    // Project perpendicular to angle (left = +90°, right = -90°) in 2D
    public static Translation3d projectPerpendicular(Translation3d base, Rotation2d angle, double distance) {
        Rotation2d perp;

        perp = angle.plus(Rotation2d.fromDegrees(90));

        return base.plus(new Translation3d(new Translation2d(distance * perp.getCos(), distance * perp.getSin())));
    }
}
