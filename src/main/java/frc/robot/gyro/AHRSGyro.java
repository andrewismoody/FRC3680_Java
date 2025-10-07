package frc.robot.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class AHRSGyro implements Gyro {

    AHRS ahrs = new AHRS(NavXComType.kUSB1);
    
    @Override
    public void calibrate() {
        // Not implemented
    }

    @Override
    public int setYawAxis(GyroAxis yaw_axis) {
        // Not implemented
        return 0;
    }

    @Override
    public int readRegister(int reg) {
        throw new UnsupportedOperationException("Unimplemented method 'readRegister'");
    }

    @Override
    public void writeRegister(int reg, int val) {
        throw new UnsupportedOperationException("Unimplemented method 'writeRegister'");
    }

    @Override
    public void reset() {
        // Not Implemented
    }

    @Override
    public void acquire() {
        throw new UnsupportedOperationException("Unimplemented method 'acquire'");
    }

    @Override
    public double getAngle() {
        return ahrs.getAngle();
    }

    @Override
    public double getRate() {
        return ahrs.getRate();
    }

    @Override
    public GyroAxis getYawAxis() {
        return GyroAxis.kZ;
    }

    @Override
    public double getGyroAngleX() {
        return ahrs.getRawGyroX();
    }

    @Override
    public double getGyroAngleY() {
        return ahrs.getRawGyroY();
    }

    @Override
    public double getGyroAngleZ() {
        return ahrs.getRawGyroZ();
    }

    @Override
    public double getGyroRateX() {
        // Not implemented
        return 0.0;
    }

    @Override
    public double getGyroRateY() {
        // Not implemented
        return 0.0;
    }

    @Override
    public double getGyroRateZ() {
        // Not implemented
        return 0.0;
    }

    @Override
    public double getAccelX() {
        return ahrs.getRawAccelX();
    }

    @Override
    public double getAccelY() {
        return ahrs.getRawAccelY();
    }

    @Override
    public double getAccelZ() {
        return ahrs.getRawAccelZ();
    }

    @Override
    public double getXComplementaryAngle() {
        // Not implemented
        return 0.0;
    }

    @Override
    public double getYComplementaryAngle() {
        // Not implemented
        return 0.0;
    }

    @Override
    public double getXFilteredAccelAngle() {
        // Not implemented
        return 0.0;
    }

    @Override
    public double getYFilteredAccelAngle() {
        // Not implemented
        return 0.0;
    }
    
}
