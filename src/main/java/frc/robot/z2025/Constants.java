package frc.robot.z2025;

import edu.wpi.first.math.geometry.Translation2d;
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
  
    // NEW: JSON-backed helpers (params in inches; convert to meters at use sites)
    public static double paramInches(String key, double fallbackInches) {
        return Utility.GetSeasonNumber(key, fallbackInches);
    }

    public static double paramMeters(String key, double fallbackMeters) {
        double fallbackIn = Utility.metersToInches(fallbackMeters);
        return Utility.inchesToMeters(Utility.GetSeasonNumber(key, fallbackIn));
    }

    public static Translation2d paramVec2Inches(String key, Translation2d fallbackInches) {
        return Utility.GetSeasonVec2Inches(key, fallbackInches);
    }

    public static Translation2d paramVec2Meters(String key, Translation2d fallbackMeters) {
        Translation2d fbIn = (fallbackMeters == null)
            ? null
            : new Translation2d(Utility.metersToInches(fallbackMeters.getX()), Utility.metersToInches(fallbackMeters.getY()));
        Translation2d vIn = Utility.GetSeasonVec2Inches(key, fbIn);
        if (vIn == null) return fallbackMeters;
        return new Translation2d(Utility.inchesToMeters(vIn.getX()), Utility.inchesToMeters(vIn.getY()));
    }
}
