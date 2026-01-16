package frc.robot.z2025;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.misc.Utility;

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

    public static final Translation2d blueStartPosition = new Translation2d(Constants.fieldCenter.getX() - Constants.startArea.getX() + robotOffset.getX(), Constants.fieldCenter.getY());
    // 3 ft. 5½ in., 7 ft. ⅜ in., 10 ft. 7⅜ in. (~105 cm, ~214 cm, ~324 cm) from mid field to the center 
    public static final double[] blueStartY = new double[] {
        Utility.inchesToMeters(127.38),
        Utility.inchesToMeters(84.03),
        Utility.inchesToMeters(36.458),
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
}
