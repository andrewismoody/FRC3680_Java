package frc.robot.encoder;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogAbsoluteEncoder implements Encoder {

    AnalogInput input;

    public AnalogAbsoluteEncoder(int pin) {
        input = new AnalogInput(pin);
    }

    public double getDistance() {
        return input.getValue() * 0.087890625; // 360/4096 = 0.087890625
    }

    public void setDistancePerPulse(double dpp) {
        // not implememented for absolute position
    }

    public void setReverseDirection(boolean reverse) {
        // not implememented for absolute position
    }
    
}
