package frc.robot.encoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotBase;

public class REVEncoder implements Encoder {
    
    RelativeEncoder internalRelativeEncoder;
    AbsoluteEncoder internalAbsoluteEncoder;
    double angleOffsetRad = 0.0; // angle to subtract from actual angle to zero the encoder
    double multiplier = 1.0;
    long giveUpTime = 1000;
    double value = 0.0;
    double velocity = 0.0;

    public void setMultiplier(double value) {
        multiplier = value;
    }

    public double getMultiplier() {
        return multiplier;
    }

    public REVEncoder(RelativeEncoder WrappedEncoder) {
        internalRelativeEncoder = WrappedEncoder;
    }

    public REVEncoder(RelativeEncoder WrappedEncoder, double DistancePerPulse) {
        this(WrappedEncoder);

        setDistancePerPulse(DistancePerPulse);
    }

    public REVEncoder(RelativeEncoder WrappedEncoder, double DistancePerPulse, boolean reverse) {
        this(WrappedEncoder, DistancePerPulse);
        
        setReverseDirection(reverse);
    }

    public REVEncoder(AbsoluteEncoder WrappedEncoder) {
        internalAbsoluteEncoder = WrappedEncoder;
    }

    public REVEncoder(AbsoluteEncoder WrappedEncoder, double DistancePerPulse) {
        this(WrappedEncoder);

        setDistancePerPulse(DistancePerPulse);
    }

    public REVEncoder(AbsoluteEncoder WrappedEncoder, double DistancePerPulse, boolean reverse) {
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

    public void appendSimValueRad(double angleRad) {
        var previousValue = value;
        // REV Encoder reports rotations, not radians or degrees
        value += angleRad / (Math.PI * 2);
        velocity = (value - previousValue) / (0.02 * 60); // assume 20ms loop, result is RPM
        //System.out.printf("(%f - %f) [%f] / %f * %f [%f] = %f\n", value, previousValue, value - previousValue, 0.02, 60.0, 0.02 * 60, velocity);
    }

    public void appendSimValueRot(double angle) {
        var previousValue = value;
        // REV Encoder reports rotations, not radians or degrees
        value += angle;
        velocity = (value - previousValue) / (0.02 * 60); // assume 20ms loop, result is RPM
        //System.out.printf("(%f - %f) [%f] / %f * %f [%f] = %f\n", value, previousValue, value - previousValue, 0.02, 60.0, 0.02 * 60, velocity);
    }

    public double getVelocity() {
        if (RobotBase.isReal()) {
            if (internalRelativeEncoder != null)
                velocity = internalRelativeEncoder.getVelocity();
            else if (internalAbsoluteEncoder != null)
                velocity = internalAbsoluteEncoder.getVelocity();
        }

        return velocity;
    }

    public double getRawValue() {
        if (RobotBase.isReal()) {
            // REV Encoder reports rotations, not radians or degrees
            if (internalRelativeEncoder != null)
                value = internalRelativeEncoder.getPosition() * multiplier + (angleOffsetRad / (Math.PI * 2));
            else if (internalAbsoluteEncoder != null)
                value = internalAbsoluteEncoder.getPosition() * multiplier + (angleOffsetRad / (Math.PI * 2));
        }

        return value;
    }

    public double getDistanceDeg() {
        return getRawValue() * 360.0 % 360;
    }

    public void setZeroPosition() {
        if (internalRelativeEncoder == null)
            return;

        value = angleOffsetRad;
        var result = internalRelativeEncoder.setPosition(angleOffsetRad);
        var sleepTime = 10L;
        var elapsedTime = 0L;
        while (elapsedTime < giveUpTime && internalRelativeEncoder.getPosition() != angleOffsetRad) {
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
        if (internalAbsoluteEncoder != null)
            return true;

        return false;
    }
}
