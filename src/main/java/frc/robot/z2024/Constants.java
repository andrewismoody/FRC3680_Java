package frc.robot.z2024;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.action.Group;
import frc.robot.misc.Utility;
import frc.robot.z2024.action.Location;

public class Constants {
    public static final double floatTolerance = 0.04; // 0.2;
    // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 / 60 = 94.6 rotations per second
    // 100:1 gearbox on 94.6 rps = 0.946 rps shaft output
    public static final double shootSpeed = (5676.0 / 60.0) / 100.0;
    public static final double feedSpeed = (5676.0 / 60.0) / 100.0;
    public static final double pickupSpeed = (5676.0 / 60.0) / 100.0;
 
    public static final double divider = 0.5;
    public static final double speedMod = 1.0;
  
    public static final double driveGearRatio = 27.0;
    // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 / 60 = 94.6 rotations per second
    public static final double driveMotorRPM = 5676.0;
     // 4" wheel = 0.1016m diameter
     // 3.625" = 0.09271
    public static final double wheelDiameter = Utility.inchesToMeters(3.5625);
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
  
    public static final double shootDistancePerRotation = 0.0;
    public static final double feedDistancePerRotation = 0.0;
    public static final double pickupDistancePerRotation = 0.0;
  
    // Field Dimensions Y (width) = 8.05, X (length) = 17.55; 
    public static final Translation2d fieldSize = new Translation2d(16.54, 8.21);
    public static final Translation2d fieldCenter = new Translation2d(fieldSize.getX() / 2.0, fieldSize.getY() / 2.0);
    // starting line is at X = 7.56m; barge is 3.72m wide
    public static final Translation2d startArea = new Translation2d(Utility.inchesToMeters(76.125), fieldSize.getY());

    // 'wide' edge is front
    // width and length of robot frame (excluding bumpers)
    public static final Translation2d frameSize = new Translation2d(Utility.inchesToMeters(27.5), Utility.inchesToMeters(32.375));
    public static final Translation2d frameCenter = new Translation2d(frameSize.getX() / 2, frameSize.getY() / 2);
    public static final Translation2d motorOffset = new Translation2d(Utility.inchesToMeters(4.0), Utility.inchesToMeters(4.5));
    public static final Translation2d motorPosition = frameCenter.minus(motorOffset);

    public static final double bumperWidth = Utility.inchesToMeters(4);
    public static final Translation2d robotSize = frameSize.plus(new Translation2d(bumperWidth, bumperWidth));
    public static final Translation2d robotCenter = new Translation2d(robotSize.getX() / 2, robotSize.getY() / 2);

    public static final double startPadding = (Constants.startArea.getX() / 2 - Constants.robotCenter.getX());
    public static final Translation2d robotOffset = robotCenter.plus(new Translation2d(startPadding, startPadding * 2));

    public static final Translation2d blueStartPosition = new Translation2d(robotOffset.getX(), 0.0);
    // 3 ft. 5½ in., 7 ft. ⅜ in., 10 ft. 7⅜ in. (~105 cm, ~214 cm, ~324 cm) from mid field to the center 
    public static final double[] blueStartY = new double[] {
        Constants.fieldCenter.getY() - Utility.inchesToMeters(36),
        Utility.inchesToMeters(144),
        Utility.inchesToMeters(72),
    };

    public static final double waypointOffset = robotSize.getNorm() * 1.25;
    public static final double alignOffset = robotSize.getNorm() * 0.75;
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

    public static final HashMap<Integer, Integer> stageIndexToTag = new HashMap<Integer, Integer>() {
        {
            put(1, 15);
            put(2, 16);
            put(3, 14);
        }
    };

    public static final HashMap<Integer, Integer> speakerIndexToTag = new HashMap<Integer, Integer>() {
        {
            put(1, 7);
            put(2, 8);
        }
    };

    public static final HashMap<Integer, Integer> sourceIndexToTag = new HashMap<Integer, Integer>() {
        {
            put(1, 9);
            put(2, 10);
        }
    };

    public static Translation3d getBlueStartFieldPosition(Group group, Location location, int index) {
        Translation3d selectedLocation = Translation3d.kZero;
        var stageTag = stageIndexToTag.get(index);
        var speakerTag = speakerIndexToTag.get(index);
        var sourceTag = sourceIndexToTag.get(index);

        switch (location) {
            case Start:
                selectedLocation = getStartPose(index).getTranslation();
            case Waypoint:
                switch (index) {
                    case 1:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, 15), getBlueStartKnownRotation(Group.Any, Location.Tag, 15), waypointOffset);
                        break;
                    case 3:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, 16), getBlueStartKnownRotation(Group.Any, Location.Tag, 16), waypointOffset);
                        break;
                    case 5:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, 14), getBlueStartKnownRotation(Group.Any, Location.Tag, 14), waypointOffset);
                        break;
                    case 2:
                        Pose2d bisectorAngle2 = Utility.perpendicularBisectorAngle(getBlueStartFieldPosition2d(Group.Any, Location.Waypoint, 1), getBlueStartFieldPosition2d(Group.Any, Location.Waypoint, 3));
                        // System.out.printf("bisector 2: (%f, %f)@%f\n", Utility.metersToInches(bisectorAngle2.getX()), Utility.metersToInches(bisectorAngle2.getY()), bisectorAngle2.getRotation().getDegrees());
                        selectedLocation = Utility.projectParallel(new Translation3d(Utility.getIntersection(getBlueStartFieldPose(Group.Any, Location.Waypoint, 1), getBlueStartFieldPose(Group.Any, Location.Waypoint, 3))), bisectorAngle2.getRotation(), waypointOffset);
                        break;
                    case 4:
                        Pose2d bisectorAngle4 = Utility.perpendicularBisectorAngle(getBlueStartFieldPosition2d(Group.Any, Location.Waypoint, 3), getBlueStartFieldPosition2d(Group.Any, Location.Waypoint, 5));
                        // System.out.printf("bisector 4: (%f, %f)@%f\n", Utility.metersToInches(bisectorAngle4.getX()), Utility.metersToInches(bisectorAngle4.getY()), bisectorAngle4.getRotation().getDegrees());
                        selectedLocation = Utility.projectParallel(new Translation3d(Utility.getIntersection(getBlueStartFieldPose(Group.Any, Location.Waypoint, 3), getBlueStartFieldPose(Group.Any, Location.Waypoint, 5))), bisectorAngle4.getRotation(), waypointOffset);
                        break;
                    case 6:
                        Pose2d bisectorAngle6 = Utility.perpendicularBisectorAngle(getBlueStartFieldPosition2d(Group.Any, Location.Waypoint, 5), getBlueStartFieldPosition2d(Group.Any, Location.Waypoint, 1));
                        // System.out.printf("bisector 6: (%f, %f)@%f\n", Utility.metersToInches(bisectorAngle6.getX()), Utility.metersToInches(bisectorAngle6.getY()), bisectorAngle6.getRotation().getDegrees());
                        selectedLocation = Utility.projectParallel(new Translation3d(Utility.getIntersection(getBlueStartFieldPose(Group.Any, Location.Waypoint, 5), getBlueStartFieldPose(Group.Any, Location.Waypoint, 1))), bisectorAngle6.getRotation(), waypointOffset);
                        break;
                    default:
                        break;
                }
                break;
            case Stage:
                switch (group) {
                    case ApproachLeft:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, stageTag), getBlueStartKnownRotation(Group.Any, Location.Tag, stageTag), robotCenter.getY());
                        selectedLocation = Utility.projectParallel(selectedLocation, getBlueStartKnownRotation(Group.Any, Location.Tag, stageTag), alignOffset);
                        break;
                    case AlignLeft:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.ApproachLeft, Location.Stage, index), getBlueStartKnownRotation(Group.Any, Location.Tag, stageTag), robotCenter.getNorm());
                        break;
                    case ApproachRight:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, stageTag), getBlueStartKnownRotation(Group.Any, Location.Tag, stageTag), robotCenter.getY());
                        selectedLocation = Utility.projectParallel(selectedLocation, getBlueStartKnownRotation(Group.Any, Location.Tag, stageTag), -alignOffset);
                        break;
                    case AlignRight:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.ApproachRight, Location.Stage, index), getBlueStartKnownRotation(Group.Any, Location.Tag, stageTag), robotCenter.getNorm());
                        break;
                    default:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, stageTag), getBlueStartKnownRotation(Group.Any, Location.Tag, stageTag), robotCenter.getY());
                        selectedLocation = Utility.projectParallel(selectedLocation, getBlueStartKnownRotation(Group.Any, Location.Tag, stageTag), index % 2 == 0 ? -scoreOffset : scoreOffset);
                        break;
                }
                break;
            case Speaker:
                switch (group) {
                    case Align:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Speaker, index), getBlueStartKnownRotation(Group.Any, Location.Tag, speakerTag), alignOffset);
                        break;
                    default:
                        selectedLocation =  Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, speakerTag), getBlueStartKnownRotation(Group.Any, Location.Tag, speakerTag), robotCenter.getY());
                        break;
                }
                break;
                case Source:
                switch (group) {
                    case Align:
                        selectedLocation = Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Source, index), getBlueStartKnownRotation(Group.Any, Location.Tag, sourceTag), alignOffset);
                        break;
                    default:
                        selectedLocation =  Utility.projectPerpendicular(getBlueStartFieldPosition(Group.Any, Location.Tag, sourceTag), getBlueStartKnownRotation(Group.Any, Location.Tag, sourceTag), robotCenter.getY());
                        break;
                }
                break;
            case Tag:
                switch (index) {
                    case 6:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(72.5),  Utility.inchesToMeters(323), 0);
                        break;
                    case 7:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(-1.5),  Utility.inchesToMeters(218.42), 0);
                        break;
                    case 8:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(-1.5), Utility.inchesToMeters(196.17), 0);
                        break;
                    case 9:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(14.02), Utility.inchesToMeters(34.79), 0);
                        break;
                    case 10:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(57.54), Utility.inchesToMeters(9.68), 0);
                        break;
                    case 14:
                        selectedLocation = new Translation3d(Utility.inchesToMeters(209.48), Utility.inchesToMeters(161.62), 0);
                        break;
                    case 15:
                        selectedLocation = new Translation3d(Utility.inchesToMeters( 182.73), Utility.inchesToMeters(177.10), 0);
                        break;
                    case 16:
                        selectedLocation = new Translation3d(Utility.inchesToMeters( 182.73), Utility.inchesToMeters(146.19), 0);
                        break;
                }
                break;
            case Interest:
                switch (index) {
                    case 1:
                        // center of stage structure
                        selectedLocation = new Translation3d(Utility.inchesToMeters(Utility.inchesToMeters(121.0)), fieldCenter.getY(), 0);
                        break;
                    case 2:
                        // central speaker scoring position
                        var interest2Bisector = Utility.perpendicularBisectorAngle(getBlueStartFieldPosition2d(group, Location.Tag, 7), getBlueStartFieldPosition2d(group, Location.Tag, 8));
                        selectedLocation = Utility.projectParallel(new Translation3d(Utility.getIntersection(getBlueStartFieldPose(group, Location.Tag, 7), getBlueStartFieldPose(group, Location.Tag, 8))), interest2Bisector.getRotation(), alignOffset);
                        break;
                }
                break;
            case Any:
            case None:
            case AdHoc:
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
                    case 6:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(270)).minus(alignmentRotation);
                        break;
                    case 7:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(0)).minus(alignmentRotation);
                        break;
                    case 8:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(0)).minus(alignmentRotation);
                        break;
                    case 9:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(60)).minus(alignmentRotation);
                        break;
                    case 10:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(60)).minus(alignmentRotation);
                        break;
                    case 14:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(0)).minus(alignmentRotation);
                        break;
                    case 15:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(120)).minus(alignmentRotation);
                        break;
                    case 16:
                        selectedRotation = new Rotation2d(Utility.degreesToRadians(240)).minus(alignmentRotation);
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
            case Stage:
            case Source:
            case Speaker:
            case AdHoc:
            case Any:
            case None:
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
