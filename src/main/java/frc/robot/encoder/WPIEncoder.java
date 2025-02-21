package frc.robot.encoder;

public class WPIEncoder implements Encoder {
    
    edu.wpi.first.wpilibj.Encoder internalEncoder;
    double angleOffsetRad = 0.0; // angle to subtract from actual angle to zero the encoder

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
        return internalEncoder.getRaw();
    }

    public double getDistance() {
        // TODO: implement angle offset
        
        return internalEncoder.getDistance();
    }

    public boolean isAbsolute() {
        return false;
    }
}
