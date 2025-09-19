package frc.robot.encoder;

import com.revrobotics.RelativeEncoder;

public class REVEncoder implements Encoder {
    
    RelativeEncoder internalEncoder;
    double angleOffsetRad = 0.0; // angle to subtract from actual angle to zero the encoder

    public REVEncoder(RelativeEncoder WrappedEncoder) {
        internalEncoder = WrappedEncoder;

    }

    public REVEncoder(RelativeEncoder WrappedEncoder, double DistancePerPulse) {
        this(WrappedEncoder);

        setDistancePerPulse(DistancePerPulse);
    }

    public REVEncoder(RelativeEncoder WrappedEncoder, double DistancePerPulse, boolean reverse) {
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
        // Not Implemented
        // internalEncoder.setDistancePerPulse(dpp);
    }

    public void setReverseDirection(boolean reverse) {
        // Not Implemented
        // internalEncoder.setReverseDirection(reverse);
    }

    public double getRawValue() {
        return getDistance();
    }

    public double getDistance() {
        return internalEncoder.getPosition() * 360.0;
    }

    public boolean isAbsolute() {
        return false;
    }
}
