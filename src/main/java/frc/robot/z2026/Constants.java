package frc.robot.z2026;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.action.Group;
import frc.robot.misc.Utility;
import frc.robot.z2026.action.Location;

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
  
    public static final double driveGearRatio = 5.08;
    // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 / 60 = 94.6 rotations per second
    public static final double driveMotorRPM = 6784.0;
     // 4" wheel = 0.1016m diameter
     // 3.625" = 0.09271
    public static final double wheelDiameter = Utility.inchesToMeters(3);
    // 4" 0.319m wheel circumference
    // 3.625" 0.291078 wheel circumference
    public static final double wheelCircumference = Math.PI * wheelDiameter;
    // 9:1 gearbox with 3:1 gear reduction (27:1 total) on 0.319m circumference = 0.0118 meters per motor rotation
    // .0107 meters per rotation
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
    public static final double steerGearRatio = 46.42;
    public static final double steerMotorRPM = 11000.0;
    public static final double steerMotorSpeed = ((steerMotorRPM * (Math.PI * 2)) / 60.0) / steerGearRatio; 
    // 20:1 gearbox
    // the encoder multiplier is only needed for motor-side encoders.  If the encoder is on the output shaft, set this to 1.0
    public static final double steeringEncoderMultiplier = 1.0; // / steerGearRatio;
  
    public static final double elevatorMaxDistance = 0.5;
    public static final double elevatorDistancePerRotation = 0.3;
  
    // Field Dimensions Y (width) = 8.07, X (length) = 16.54; 
    public static final Translation2d fieldSize = new Translation2d(16.54, 8.07);
    public static final Translation2d fieldCenter = new Translation2d(fieldSize.getX() / 2.0, fieldSize.getY() / 2.0);
    // starting line is at X = 4.03; Start is 8.07m wide (width of field)
    public static final Translation2d startArea = new Translation2d(4.03, 8.07);

    // 'wide' edge is front
    // width and length of robot frame (excluding bumpers)
    public static final Translation2d frameSize = new Translation2d(Utility.inchesToMeters(27), Utility.inchesToMeters(27));
    public static final Translation2d frameCenter = new Translation2d(frameSize.getX() / 2, frameSize.getY() / 2);
    // motorOffset is the distance from the edge of the frame to the center of the drive module wheel rotation axis
    public static final Translation2d motorOffset = new Translation2d(Utility.inchesToMeters(1.81), Utility.inchesToMeters(1.81));
    // motorPosition is the distance from the center of the frame to the center of the drive module wheel rotation axis
    public static final Translation2d motorPosition = frameCenter.minus(motorOffset);

    public static final double bumperWidth = Utility.inchesToMeters(4);
    public static final Translation2d robotSize = frameSize.plus(new Translation2d(bumperWidth, bumperWidth));
    public static final Translation2d robotCenter = new Translation2d(robotSize.getX() / 2, robotSize.getY() / 2);

    public static final double startPadding = (Constants.startArea.getX() / 2 - Constants.robotCenter.getX());
    public static final Translation2d robotOffset = robotCenter.plus(new Translation2d(startPadding, startPadding * 2));

    public static final Translation2d blueStartPosition = new Translation2d(Constants.fieldCenter.getX() - Constants.startArea.getX() + robotOffset.getX(), Constants.fieldCenter.getY());
    // 3 ft. 5½ in., 7 ft. ⅜ in., 10 ft. 7⅜ in. (~105 cm, ~214 cm, ~324 cm) from mid field to the center 
    public static final double[] blueStartY = new double[] {
        Utility.inchesToMeters(127.38),
        Utility.inchesToMeters(84.03),
        Utility.inchesToMeters(36.458),
    };

    public static final double waypointOffset = robotSize.getNorm() * 1.25;
    public static final double shootingDistance = Utility.inchesToMeters(24);
    public static final double approachOffset = (robotSize.getY() * 0.5) + shootingDistance;
    public static final double towerDistance = Utility.inchesToMeters(49.32);
    public static final double towerApproachOffset = (robotSize.getNorm()) + towerDistance;
    public static final double towerAlignOffset = (robotSize.getY() * 0.5) + towerDistance;
    public static final double scoreOffset = Utility.inchesToMeters(-3);

    public static Pose3d getMyStartPose() {
        var myLocation = Utility.getDriverLocation();
        return getStartPose(myLocation);
    }
    
    public static Pose3d getStartPose(int index) {
        if (index < 1)
            index = 1;
        var thisStartPosition = Constants.blueStartPosition.plus(new Translation2d(0.0, blueStartY[index - 1]));
        var transformedPosition = Utility.transformToAllianceStart(new Translation3d(thisStartPosition));
        return new Pose3d(transformedPosition, Rotation3d.kZero);
    }

    public static final HashMap<Integer, Integer> HubIndexToTag = new HashMap<Integer, Integer>() {
        {
            put(1, 18);
            put(2, 20);
            put(3, 21);
            put(4, 26);
        }
    };

    public static final HashMap<Integer, Integer> OutpostIndexToTag = new HashMap<Integer, Integer>() {
        {
            put(1, 29);
        }
    };

    public static final HashMap<Integer, Integer> TowerIndexToTag = new HashMap<Integer, Integer>() {
        {
            put(1, 31);
        }
    };

    public static final HashMap<Integer, Integer> TrenchIndexToTag = new HashMap<Integer, Integer>() {
        {
            put(1, 17);
            put(2, 22);
            put(3, 23);
            put(4, 28);
        }
    };

    // TODO: define approaches and positions for scoring, etc
    public static Translation3d getBlueStartFieldPosition(Group group, Location location, int index) {
        Translation3d selectedLocation = Translation3d.kZero;
        var HubTag = HubIndexToTag.get(index);
        var OutpostTag = OutpostIndexToTag.get(index);
        var TowerTag = TowerIndexToTag.get(index);

        switch (location) {
            case Start:
                selectedLocation = getStartPose(index).getTranslation();
            case Waypoint:
                switch (index) {
                    case 1:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(4.85), Utility.inchesToMeters(78.03), 0);
                        break;
                    case 2:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(98.36), Utility.inchesToMeters(78.03), 0);
                        break;
                    case 3:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(158.32), Utility.inchesToMeters(78.03), 0);
                        break;
                    case 4:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(218.28), Utility.inchesToMeters(78.03), 0);
                        break;
                    case 5:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(311.79), Utility.inchesToMeters(78.03), 0);
                        break;
                    case 6:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(4.85), Utility.inchesToMeters(261.76), 0);
                        break;
                    case 7:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(98.36), Utility.inchesToMeters(261.76), 0);
                        break;
                    case 8:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(158.32), Utility.inchesToMeters(261.76), 0);
                        break;
                    case 9:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(218.28), Utility.inchesToMeters(261.76), 0);
                        break;
                    case 10:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(311.79), Utility.inchesToMeters(261.76), 0);
                        break;
                    default:
                        break;
                }
                break;
            case Hub:
                selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, HubTag), getBlueStartKnownRotation(Group.Any, Location.Tag, HubTag), approachOffset);
                break;
            case Outpost:
                switch(group) {
                    case Align:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, OutpostTag), getBlueStartKnownRotation(Group.Any, Location.Tag, OutpostTag), robotSize.getY());
                        break;
                    case Approach:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, OutpostTag), getBlueStartKnownRotation(Group.Any, Location.Tag, OutpostTag), approachOffset);
                        break;
                    default:
                        break;
                }
                break;
            case Tag:
                switch (index) {
                    case 17:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(183.03), Utility.inchesToMeters(24.85), 0);
                        break;
                    case 18:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(181.56), Utility.inchesToMeters(134.56), 0);
                        break;
                    case 19:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(205.32), Utility.inchesToMeters(144.32), 0);
                        break;
                    case 20:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(205.32), Utility.inchesToMeters(158.32), 0);
                        break;
                    case 21:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(181.56), Utility.inchesToMeters(182.08), 0);
                        break;
                    case 22:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(183.03), Utility.inchesToMeters(291.79), 0);
                        break;
                    case 23:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(180.08), Utility.inchesToMeters(291.79), 0);
                        break;
                    case 24:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(167.56), Utility.inchesToMeters(182.08), 0);
                        break;
                    case 25:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(157.79), Utility.inchesToMeters(172.32), 0);
                        break;
                    case 26:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(157.79), Utility.inchesToMeters(158.32), 0);
                        break;
                    case 27:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(167.56), Utility.inchesToMeters(134.56), 0);
                        break;
                    case 28:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(180.08), Utility.inchesToMeters(24.85), 0);
                        break;
                    case 29:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(0.54), Utility.inchesToMeters(25.62), 0);
                        break;
                    case 30:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(0.54), Utility.inchesToMeters(42.62), 0);
                        break;
                    case 31:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(0.55), Utility.inchesToMeters(146.86), 0);
                        break;
                    case 32:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(0.55), Utility.inchesToMeters(163.86), 0);
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
            case Tower:
                switch(group) {
                    case Align:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, TowerTag), getBlueStartKnownRotation(Group.Any, Location.Tag, TowerTag), towerAlignOffset);
                        break;
                    case Approach:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, TowerTag), getBlueStartKnownRotation(Group.Any, Location.Tag, TowerTag), towerApproachOffset);
                        break;
                    default:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, TowerTag), getBlueStartKnownRotation(Group.Any, Location.Tag, TowerTag), towerApproachOffset);
                        break;
                }
                break;
            case Any:
            case None:
            case Depot:
            case AdHoc:
            case Bump:
            case Trench:
            default:
                break;
        }

        return selectedLocation;
    }

    public static Translation2d getBlueStartFieldPosition2d(Group group, Location location, int index) {
        return getBlueStartFieldPosition(group, location, index).toTranslation2d();
    }

    public static Translation3d getFieldPosition(Group group, Location location, int index) {
        var position = getBlueStartFieldPosition(group, location, index);

        return Utility.transformToAllianceStart(position);
    }

    public static Translation2d getFieldPosition2d(Group group, Location location, int index) {
        return getFieldPosition(group, location, index).toTranslation2d();
    }

    public static Translation3d getRedStartFieldPosition(Group group, Location location, int index) {
        var position = getBlueStartFieldPosition(group, location, index);

        return Utility.transformToRedStart(position);
    }

    public static Translation2d getRedStartFieldPosition2d(Group group, Location location, int index) {
        return getRedStartFieldPosition(group, location, index).toTranslation2d();
    }

    public static Rotation2d getBlueStartKnownRotation(Group group, Location location, int index) {
        var selectedRotation = Rotation2d.kZero;
        var alignmentRotation = Rotation2d.fromDegrees(90);

        switch (location) {
            case Tag:
                switch (index) {
                    case 17:
                        selectedRotation = new Rotation2d().minus(alignmentRotation);
                        break;
                    case 18:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(270)).minus(alignmentRotation);
                        break;
                    case 19:
                        selectedRotation = new Rotation2d().minus(alignmentRotation);
                        break;
                    case 20:
                        selectedRotation = new Rotation2d().minus(alignmentRotation);
                        break;
                    case 21:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(90)).minus(alignmentRotation);
                        break;
                    case 22:
                        selectedRotation = new Rotation2d().minus(alignmentRotation);
                        break;
                    case 23:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(180)).minus(alignmentRotation);
                        break;
                    case 24:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(90)).minus(alignmentRotation);
                        break;
                    case 25:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(180)).minus(alignmentRotation);
                        break;
                    case 26:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(180)).minus(alignmentRotation);
                        break;
                    case 27:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(270)).minus(alignmentRotation);
                        break;
                    case 28:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(180)).minus(alignmentRotation);
                        break;
                    case 29:
                        selectedRotation = new Rotation2d().minus(alignmentRotation);
                        break;
                    case 30:
                        selectedRotation = new Rotation2d().minus(alignmentRotation);
                        break;
                    case 31:
                        selectedRotation = new Rotation2d().minus(alignmentRotation);
                        break;
                    case 32:
                        selectedRotation = new Rotation2d().minus(alignmentRotation);
                        break;
                }
                break;
            case Interest:
                switch (index) {
                    case 1:
                        selectedRotation = new Rotation2d(2.75);
                    break;
                }
                break;
            case Start:
            case Waypoint:
            case Hub:
            case Depot:
            case Outpost:
            case AdHoc:
            case Any:
            case None:
            case Bump:
            case Tower:
            case Trench:
            default:
                break;
        }

        return selectedRotation;
    }

    public static Rotation2d getKnownRotation(Group group, Location location, int index) {
        var rotation = getBlueStartKnownRotation(group, location, index);

        return Utility.rotateToAllianceStart(rotation);
    }

    public static Rotation2d getRedStartKnownRotation(Group group, Location location, int index) {
        var rotation = getBlueStartKnownRotation(group, location, index);

        return Utility.rotateToRedStart(rotation);
    }

    public static Pose2d getBlueStartFieldPose(Group group, Location location, int index) {
        return new Pose2d(getBlueStartFieldPosition2d(group, location, index), getBlueStartKnownRotation(group, location, index));
    }

    public static Pose2d getRedStartFieldPose(Group group, Location location, int index) {
        return new Pose2d(getRedStartFieldPosition2d(group, location, index), getRedStartKnownRotation(group, location, index));
    }

    public static Pose2d getFieldPose(Group group, Location location, int index) {
        return new Pose2d(getFieldPosition2d(group, location, index), getKnownRotation(group, location, index));
    }
}
