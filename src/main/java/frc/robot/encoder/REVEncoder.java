package frc.robot.encoder;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class REVEncoder implements Encoder {
    
    RelativeEncoder internalEncoder;
    double angleOffsetRad = 0.0; // angle to subtract from actual angle to zero the encoder
    double multiplier = 1.0;
    long giveUpTime = 1000;

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
        // Not Implemented - for SparkMAX, must be set on the motor controller itself
        // internalEncoder.setReverseDirection(reverse);
    }

    public double getAngleOffsetRad() {
        return angleOffsetRad;
    }

    public double getRawValue() {
        return internalEncoder.getPosition() * multiplier + angleOffsetRad;
    }

    public double getDistanceDeg() {
        return getRawValue() * 360.0 % 360;
    }

    public void setZeroPosition() {
        var result = internalEncoder.setPosition(angleOffsetRad);
        var sleepTime = 10L;
        var elapsedTime = 0L;
        while (elapsedTime < giveUpTime && internalEncoder.getPosition() != angleOffsetRad) {
            try {
                Thread.sleep(sleepTime);
                elapsedTime += sleepTime;
            } catch (Exception e) {

            }
        }

        if (result != REVLibError.kOk)
            System.out.println("Set REV Encoder Zero Position Result: " + result);
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
