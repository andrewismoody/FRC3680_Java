package frc.robot.encoder;

import edu.wpi.first.wpilibj.RobotBase;

public class QuadEncoder implements Encoder {
    
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

    public QuadEncoder(int pinA, int pinB) {
        internalEncoder = new edu.wpi.first.wpilibj.Encoder(pinA, pinB);

        internalEncoder.reset();
    }

    public QuadEncoder(int pinA, int pinB, double DistancePerPulse) {
        this(pinA, pinB);

        setDistancePerPulse(DistancePerPulse);
    }

    public QuadEncoder(int pinA, int pinB, double DistancePerPulse, boolean reverse) {
        this(pinA, pinB, DistancePerPulse);
        
        setReverseDirection(reverse);
    }

    public double getVelocity() {
        // Not implemented
        return 0.0;
    }

    public boolean isAbsolute() {
        return false;
    }

    public void setAngleOffsetDeg(double value) {
        setAngleOffsetRad(value * 0.0174532);
    }

    public void setAngleOffsetRad(double value) {
        angleOffsetRad = value;
    }

    public void setDistancePerPulse(double dpp) {
        internalEncoder.setDistancePerPulse(dpp);
    }

    public void setReverseDirection(boolean reverse) {
        internalEncoder.setReverseDirection(reverse);
    }

    public double getRawValue() {
        // TODO not sure raw value should be modifying the output
        if (RobotBase.isReal())
            value = internalEncoder.getRaw() * multiplier;

        return value * multiplier;
    }

    public void appendSimValueRad(double angleRad) {
        value += angleRad;
    }

    public double getDistance() {
        // TODO: implement angle offset
        
        if (RobotBase.isReal())
            value = internalEncoder.getDistance();

        return value;
    }

    public double getAngleOffsetRad() {
        return angleOffsetRad;
    }

    public void setZeroPosition() {
        // TODO: check if raw value should be rad or deg
        setAngleOffsetRad(-getRawValue());
    }
}
