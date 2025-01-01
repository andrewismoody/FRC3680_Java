package frc.robot.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogAbsoluteEncoder implements Encoder {

    AnalogInput input;
    double resolution = 4096; // range of bytes returned by the encoder
    double value = 0;
    int mypin = -1;
    double angleOffsetRad = 0.0; // angle to subtract from actual angle to zero the encoder

    public AnalogAbsoluteEncoder(int pin) {
        input = new AnalogInput(pin);
        mypin = pin;
    }

    public void setAngleOffsetDeg(double value) {
        setAngleOffsetRad(value * 0.0174532);
    }

    public void setAngleOffsetRad(double value) {
        angleOffsetRad = value;
    }

    public double getDistance() {
        boolean reportRadians = false;

        value = input.getValue();
        double returnValue = 0;

        returnValue = value * (6.28 / resolution); // 6.28 / 4096 = 0.001533203125 (radians)
        Rotation2d valueRot = Rotation2d.fromRadians(returnValue);

        valueRot = valueRot.minus(Rotation2d.fromRadians(angleOffsetRad));

        if (reportRadians)
            returnValue = valueRot.getRadians();
        else
            returnValue = valueRot.getDegrees();
            
        //System.out.printf("pin %d, encoder value: %f\n", mypin, returnValue);

        return returnValue;
    }

    public void setDistancePerPulse(double dpp) {
        // not implememented for absolute position
    }

    public void setReverseDirection(boolean reverse) {
        // not implememented for absolute position
    }

}
