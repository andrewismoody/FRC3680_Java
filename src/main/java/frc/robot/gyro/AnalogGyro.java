package frc.robot.gyro;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogGyro implements Gyro {
    double angle;
    GyroAxis yawAxis = GyroAxis.kZ;
    int analogPin;
    AnalogInput input;
    int value;

    public AnalogGyro(int AnalogPin) {
        analogPin = AnalogPin;
        input = new AnalogInput(AnalogPin);
    }

    public void calibrate() {
        return;
    }

    public int setYawAxis(GyroAxis yaw_axis) {
        yawAxis = yaw_axis;
        return 0;
    }
  
    public int readRegister(final int reg) {
        return 0;
    }
  
    public void writeRegister(final int reg, final int val) {
        return;
    }
  
    public void reset() {
        angle = 0;
    }
  
    public void acquire() {
        return;
    }
  
    public double getAngle() {
        value = input.getValue();

        angle = (value - 2048) * (360.0 / 2048); // convert from 12-bit to degrees

        return angle;
    }

    @Override
    public void appendGyroSimValue(double angleDegrees) {
        angle += ((angleDegrees % 360.0) + 360.0) % 360.0;
    } 

    public double getRate() {
        return 0;
    }
  
    public GyroAxis getYawAxis() {
        return yawAxis;
    }
  
    public double getGyroAngleX() {
        return 0;
    }
  
    public double getGyroAngleY() {
        return 0;
    }
  
    public double getGyroAngleZ() {
        return getAngle();
    }
  
    public double getGyroRateX() {
        return 0;
    }
  
    public double getGyroRateY() {
        return 0;
    }
  
    public double getGyroRateZ() {
        return 0;
    }
  
    public double getAccelX() {
        return 0;
    }
  
    public double getAccelY() {
        return 0;
    }
  
    public double getAccelZ() {
        return 0;
    }
  
    public double getXComplementaryAngle() {
        return 0;
    }
  
    public double getYComplementaryAngle() {
        return 0;
    }
  
    public double getXFilteredAccelAngle() {
        return 0;
    }
  
    public double getYFilteredAccelAngle() {
        return 0;
    }
  
}
