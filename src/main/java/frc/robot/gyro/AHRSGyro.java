package frc.robot.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.RobotBase;

public class AHRSGyro implements Gyro {

    AHRS ahrs = new AHRS(NavXComType.kUSB1);
    // private long lastAngleGetTs = 0L;
    private double lastAngle = 0.0;
    
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
        if (RobotBase.isReal())
            lastAngle = -ahrs.getAngle();
        // NavX2 is left-hand CW positive, we want right-hand CCW positive to match WPILib coordinate systems
        lastAngle = ((lastAngle % 360.0) + 360.0) % 360.0;

        return lastAngle;
    }

    @Override
    public void appendGyroSimValue(double angleDegrees) {
        // we invert the 'real' gyro value, but we not the sim here, because we always want CCW positive responsed from the gyro
        lastAngle += ((angleDegrees % 360.0) + 360.0) % 360.0;
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
