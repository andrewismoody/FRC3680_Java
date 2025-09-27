package frc.robot.encoder;

import com.revrobotics.RelativeEncoder;

public class REVEncoder implements Encoder {
    
    RelativeEncoder internalEncoder;
    double angleOffsetRad = 0.0; // angle to subtract from actual angle to zero the encoder
    double multiplier = 1.0;

    public void setMultiplier(double value) {
        multiplier = value;
    }

    public double getMultiplier() {
        return multiplier;
    }

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

    public double getAngleOffsetRad() {
        return angleOffsetRad;
    }

    public double getRawValue() {
        return internalEncoder.getPosition() * multiplier;
    }

    public double getDistanceDeg() {
        return getRawValue() * 360.0 % 360;
    }

    public void setZeroPosition() {
        var result = internalEncoder.setPosition(0.0);
        System.out.println("Set REV Encoder Zero Position Result: " + result);
        while (internalEncoder.getPosition() != 0.0) {
            try {
                Thread.sleep(10);
            } catch (Exception e) {

            }
        }
        System.out.println("Rev Encoder position set");
        //setAngleOffsetDeg(-getDistanceDeg());
    }

    public double getDistance() {
        // Rotation2d distance = Rotation2d.fromDegrees(getDistanceDeg());
        // Rotation2d offset = Rotation2d.fromRadians(angleOffsetRad);
        // return distance.plus(offset).getDegrees();
        return getDistanceDeg();
    }

    public boolean isAbsolute() {
        return false;
    }
}
