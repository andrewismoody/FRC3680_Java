package frc.robot.encoder;

import edu.wpi.first.wpilibj.RobotBase;

public class WPIEncoder implements Encoder {
    
    edu.wpi.first.wpilibj.Encoder internalEncoder;
    double angleOffsetRad = 0.0; // angle to subtract from actual angle to zero the encoder
    double multiplier = 1.0;
    double value = 0.0;

    public void setMultiplier(double value) {
        multiplier = value;
    }

    public double getMultiplier() {
        return multiplier;
    }

    public WPIEncoder(edu.wpi.first.wpilibj.Encoder WrappedEncoder) {
        internalEncoder = WrappedEncoder;

        internalEncoder.reset();
    }

    public WPIEncoder(edu.wpi.first.wpilibj.Encoder WrappedEncoder, double DistancePerPulse) {
        this(WrappedEncoder);

        setDistancePerPulse(DistancePerPulse);
    }

    public WPIEncoder(edu.wpi.first.wpilibj.Encoder WrappedEncoder, double DistancePerPulse, boolean reverse) {
        this(WrappedEncoder, DistancePerPulse);
        
        setReverseDirection(reverse);
    }

    public double getVelocity() {
        // Not implemented
        return 0.0;
    }

    public void setAngleOffsetDeg(double value) {
        setAngleOffsetRad(value * 0.0174532);
    }

    public void setAngleOffsetRad(double value) {
        angleOffsetRad = value;
    }

    public double getAngleOffsetRad() {
        return angleOffsetRad;
    }

    public void setZeroPosition() {
        // TODO: check if raw value should be rad or deg
        setAngleOffsetRad(-getRawValue());
    }

    public void setDistancePerPulse(double dpp) {
        internalEncoder.setDistancePerPulse(dpp);
    }

    public void setReverseDirection(boolean reverse) {
        internalEncoder.setReverseDirection(reverse);
    }

    public double getRawValue() {
        return internalEncoder.getRaw() * multiplier;
    }

    public void appendSimValueRad(double angleRad) {
        // REV Encoder reports rotations, not radians or degrees
        value += angleRad / (Math.PI * 2);
    }

    public double getDistance() {
        // TODO: implement angle offset
        
        if (RobotBase.isReal())
            value = internalEncoder.getDistance();

        return value;
    }

    public boolean isAbsolute() {
        return false;
    }
}
