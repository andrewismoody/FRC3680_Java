package frc.robot.z2025;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.misc.Utility;
import frc.robot.z2025.action.Location;

public class Constants {
    public static final double floatTolerance = 0.04; // 0.2;
    // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 / 60 = 94.6 rotations per second
    // 100:1 gearbox on 94.6 rps = 0.946 rps shaft output
    public static final double elevatorSpeed = (5676.0 / 60.0) / 100.0;
    public static final double elevatorEncoderMultiplier = 1.0 / 100.0;
  
    public static final double liftSpeed = 0.6;
    public static final double grabSpeed = 0.6;
  
    public static final double divider = 0.5;
    public static final double speedMod = 1.0;
  
    public static final double driveGearRatio = 27.0;
    // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 / 60 = 94.6 rotations per second
    public static final double driveMotorRPM = 5676.0;
     // 4" wheel = 0.1016m diameter
    public static final double wheelDiameter = 0.1016;
    // 0.319m wheel circumference
    public static final double wheelCircumference = Math.PI * wheelDiameter;
    // 9:1 gearbox with 3:1 gear reduction (27:1 total) on 0.319m circumference = 0.0118 meters per motor rotation
    public static final double driveRatio = wheelCircumference / driveGearRatio; 
    // m_driveSpeed should be actual meters per second that is achievable by the drive motor
    // higher numbers result in faster drive speeds. To slow it down, send a higher
    // number, which will result in a lower voltage being sent to the motor for any
    // given speed.
    // 0.0118 meters per rotation * 94.6 rotations per second = 1.116 meters per second
    public static final double driveSpeed = driveRatio * (driveMotorRPM / 60.0); 
  
    // https://cdn.andymark.com/media/W1siZiIsIjIwMjIvMDIvMDIvMDgvMzMvMTIvNzMzYmY3YmQtYTI0MC00ZDkyLWI5NGMtYjRlZWU1Zjc4NzY0L2FtLTQyMzNhIEpFLVBMRy00MTAgbW90b3IuUERGIl1d/am-4233a%20JE-PLG-410%20motor.PDF?sha=5387f684d4e2ce1f
    // higher numbers result in faster drive speeds. To slow it down, send a higher
    // number, which will result in a lower voltage being sent to the motor for any
    // given speed.
    // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 * 6.28 = 35645 radians per minute; 35645 / 60 = 594 radians per second
    // 20:1 gearbox on 594 rps = 29.7 rps shaft output
    // should be actual radians per second that is achievable by the rotation motor
    public static final double steerGearRatio = 20.0;
    public static final double steerMotorRPM = 5676.0;
    public static final double steerMotorSpeed = ((steerMotorRPM * (Math.PI * 2)) / 60.0) / steerGearRatio; 
    // 20:1 gearbox
    public static final double steeringEncoderMultiplier = 1.0 / steerGearRatio;
  
    public static final double elevatorMaxDistance = 0.5;
    public static final double elevatorDistancePerRotation = 0.3;
  
    // Field Dimensions Y (width) = 8.05, X (length) = 17.55; 
    public static final Translation2d fieldSize = new Translation2d(17.55, 8.05);
    public static final Translation2d fieldCenter = new Translation2d(fieldSize.getX() / 2.0, fieldSize.getY() / 2.0);
    // starting line is at X = 7.56m; barge is 3.72m wide
    public static final Translation2d startArea = new Translation2d(fieldSize.getX() / 2 - 7.56, 3.72);

    // flipped x and y so that 'narrow' edge is front
    public static final Translation2d frameSize = new Translation2d(0.6985, 0.822325); // meters - 32.375" x 27.5" - distance from center of robot to center of wheel

    public static final Translation2d frameCenter = new Translation2d(frameSize.getX() / 2, frameSize.getY() / 2);

    public static final double startPadding = (Constants.startArea.getX() - Constants.frameCenter.getX());
    public static final Translation2d robotOffset = frameCenter.plus(new Translation2d(startPadding, startPadding * 2));

    public static final Translation2d[] blueStartPositions = new Translation2d[] {
        new Translation2d(Constants.fieldCenter.getX() - Constants.startArea.getX() + robotOffset.getX(), Constants.fieldSize.getY() - (robotOffset.getY() * 1)),
        new Translation2d(Constants.fieldCenter.getX() - Constants.startArea.getX() + robotOffset.getX(), Constants.fieldSize.getY() - (robotOffset.getY() * 2)),
        new Translation2d(Constants.fieldCenter.getX() - Constants.startArea.getX() + robotOffset.getX(), Constants.fieldSize.getY() - (robotOffset.getY() * 3))
    };

    public static final Transform3d redStartTransform = new Transform3d(new Translation3d(Constants.fieldCenter), new Rotation3d(new Rotation2d(Math.PI)));
  
    public static Translation3d transformToRedStart(Translation3d point) {
        return point.rotateAround(Constants.redStartTransform.getTranslation(), Constants.redStartTransform.getRotation());
    }
  
    public static Rotation2d rotateToRedStart(Rotation2d angle) {
        return angle.plus(Constants.redStartTransform.getRotation().toRotation2d());
    }

    public static Pose3d rotateToRedStart(Pose3d pose) {
        return pose.rotateAround(redStartTransform.getTranslation(), redStartTransform.getRotation());
    }

    public static Translation3d transformToAllianceStart(Translation3d point) {
        if (Utility.IsRedAlliance())
            return transformToRedStart(point);
        else
            return point;
    }

    public static Rotation2d rotateToAllianceStart(Rotation2d angle) {
        if (Utility.IsRedAlliance())
            return rotateToRedStart(angle);
        else
            return angle;
    }

    public static Pose3d rotateToAllianceStart(Pose3d pose) {
        if (Utility.IsRedAlliance())
            return rotateToRedStart(pose);
        else
            return pose;
    }

    // TODO 1: fix all of these positions
    public static Translation3d getFieldPosition(Location location, int index) {
        // TODO add other locations
        Translation3d selectedLocation = Translation3d.kZero;

        switch (location) {
            case Barge:
                switch (index) {
                    case 1:
                    selectedLocation = new Translation3d(blueStartPositions[Utility.getDriverLocation() - 1]);
                    break;
                }
                break;
            case Waypoint:
                switch (index) {
                    case 1:
                        return Utility.projectPerpendicular(getFieldPosition(Location.Tag, 19), getKnownRotation(Location.Tag, 19), Utility.inchesToMeters(54));
                    case 11:
                        return Utility.projectPerpendicular(getFieldPosition(Location.Tag, 20), getKnownRotation(Location.Tag, 20), Utility.inchesToMeters(54));
                    case 12:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(176.75), Constants.fieldSize.getY() - Utility.inchesToMeters(54), 0);
                        break;
                    default:
                        break;
                }
                break;
            case Reef:
                switch (index) {
                    case 1:
                        //selectedLocation = new Translation3d(2.63, 5.66, 0);
                        var reef1 = Utility.projectPerpendicular(getFieldPosition(Location.Tag, 19), getKnownRotation(Location.Tag, 19), Utility.inchesToMeters(54));
                        return Utility.projectPerpendicular(reef1, getKnownRotation(Location.Tag, 19).plus(Rotation2d.fromDegrees(90)), Utility.inchesToMeters(-8));
                    case 2:
                        //selectedLocation = new Translation3d(2.63, 5.66, 0);
                        var reef2 = Utility.projectPerpendicular(getFieldPosition(Location.Tag, 19), getKnownRotation(Location.Tag, 19), Utility.inchesToMeters(54));
                        return Utility.projectPerpendicular(reef2, getKnownRotation(Location.Tag, 19).plus(Rotation2d.fromDegrees(90)), Utility.inchesToMeters(8));
             }
                break;
            case Coral:
                switch (index) {
                    case 1:
                        return Utility.projectPerpendicular(getFieldPosition(Location.Tag, 13), getKnownRotation(Location.Tag, 13), Utility.inchesToMeters(54));
                }
                break;
            case Tag:
                switch (index) {
                    case 13:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(33.51),  Utility.inchesToMeters(291.20), 0);
                        break;
                    case 19:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(160.39), Utility.inchesToMeters( 186.83), 0);
                        break;
                    case 20:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(193.10), Utility.inchesToMeters( 186.83), 0);
                        break;
                }
                break;
            case Interest:
                switch (index) {
                    case 1:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(176.75), Utility.inchesToMeters(158.5), 0);
                        break;
                }
                break;
            case Any:
            case None:
            case Processor:
            case AdHoc:
                break;
        }

        return transformToAllianceStart(selectedLocation);
    }

    public static Rotation2d getKnownRotation(Location location, int index) {
        var selectedRotation = Rotation2d.kZero;

        switch (location) {
            case Tag:
                switch (index) {
                    case 13:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(306)).minus(Rotation2d.fromDegrees(90));
                        break;
                    case 17:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(240)).minus(Rotation2d.fromDegrees(90));
                        break;
                    case 19:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(120)).minus(Rotation2d.fromDegrees(90));
                        break;
                    case 20:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(60)).minus(Rotation2d.fromDegrees(90));
                        break;
                    case 22:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(300)).minus(Rotation2d.fromDegrees(90));
                        break;
                    case 21:
                    case 18:
                        break;
                }
                break;
            case Barge:
            case Waypoint:
            case Reef:
            case Processor:
            case Coral:
            case AdHoc:
            case Any:
            case None:
            case Interest:
                switch (index) {
                    case 1:
                        selectedRotation = new Rotation2d(2.75);
                    break;
                }
                break;
        }

        return rotateToAllianceStart(selectedRotation);
    }
}
