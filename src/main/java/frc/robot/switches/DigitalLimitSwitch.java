package frc.robot.switches;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalLimitSwitch implements Switch {
    int pin;
    DigitalInput din;

    public DigitalLimitSwitch(int Pin) {
        pin = Pin;
        din = new DigitalInput(pin);
    }

    public boolean GetState() {
        return din.get();
    }
}
