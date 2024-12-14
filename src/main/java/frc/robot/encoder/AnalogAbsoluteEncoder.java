package frc.robot.encoder;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogAbsoluteEncoder implements Encoder {

    AnalogInput input;
    double resolution = 4096; // range of bytes returned by the encoder
    double value = 0;
    int mypin = -1;

    public AnalogAbsoluteEncoder(int pin) {
        input = new AnalogInput(pin);
        mypin = pin;
    }

    public double getDistance() {
        boolean reportRadians = false;

        value = input.getValue();
        double returnValue = 0;

        if (reportRadians)
            returnValue = value * (6.28 / resolution); // 6.28 / 4096 = 0.001533203125 (radians)
        else
            returnValue = value * (360 / resolution); // or 360/4096 = 0.087890625 (degrees)

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
