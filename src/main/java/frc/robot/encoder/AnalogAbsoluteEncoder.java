package frc.robot.encoder;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogAbsoluteEncoder implements Encoder {

    AnalogInput input;

    public AnalogAbsoluteEncoder(int pin) {
        input = new AnalogInput(pin);
    }

    public double getDistance() {
        boolean reportRadians = false;

        if (reportRadians)
            return input.getValue() * 0.001533203125; // 6.28 / 4096 = 0.001533203125 (radians)
        else
            return input.getValue() * 0.087890625; // or 360/4096 = 0.087890625 (degrees)
    }

    public void setDistancePerPulse(double dpp) {
        // not implememented for absolute position
    }

    public void setReverseDirection(boolean reverse) {
        // not implememented for absolute position
    }

}
