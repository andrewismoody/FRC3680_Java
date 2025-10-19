package frc.robot.encoder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.misc.Utility;
import edu.wpi.first.wpilibj.RobotBase;

public class I2CAbsoluteEncoder implements Encoder {
    private I2C myI2c;
    double angleOffsetRad = 0.0; // angle to subtract from actual angle to zero the encoder
    double multiplier = 1.0;
    double value = 0.0;

    public void setMultiplier(double value) {
        multiplier = value;
    }

    public double getMultiplier() {
        return multiplier;
    }

    public I2CAbsoluteEncoder() {
        this(Port.kOnboard, 0x36);
    }

    public I2CAbsoluteEncoder(int myAddr) {
        this(Port.kOnboard, myAddr);
    }

    public I2CAbsoluteEncoder(Port myPort) {
        this(myPort, 0x36);
    }

    public I2CAbsoluteEncoder(Port myPort, int myAddr) {

        myI2c = new I2C(myPort, myAddr);
    }

    public double getAngleOffsetRad() {
        return angleOffsetRad;
    }

    public void setZeroPosition() {
        // TODO: check if raw value should be rad or deg
        setAngleOffsetRad(-getRawValue());
    }

    public void setAngleOffsetDeg(double value) {
        setAngleOffsetRad(value * 0.0174532);
    }

    public void setAngleOffsetRad(double value) {
        angleOffsetRad = value;
    }

    // getRawValue doesn't support sim values, use getDistance
    public double getRawValue() {
        return readRegister(0x0C) * multiplier;                        // combine bytes to get 12-bit value 11:0
    }

    public void appendSimValueRad(double angleRad) {
        value += angleRad;
    }

    public double getDistance() {
        boolean reportRadians = false;

        if (RobotBase.isReal()) {
            double rawAngle = getRawValue();

            // TODO: implement angle offset
            
            value = rawAngle * 0.001533203125; // 6.28 / 4096 = 0.001533203125 (radians)
        }

        if (reportRadians)
            return value;
        else
            return Utility.radiansToDegrees(value);
    }

    public void setDistancePerPulse(double dpp) {
        // not implememented for absolute position
    }

    public void setReverseDirection(boolean reverse) {
        // not implememented for absolute position
    }

    public static int toUShort(byte[] buf) {
        return buf[0] << 8 | buf[1];
    }

    public int readRegister(final int reg) {
        byte[] buf = new byte[2];

        myI2c.read(reg, 2, buf);

        return toUShort(buf);
    }
  
    public void writeRegister(final int reg, final int val) {
        myI2c.write(reg, val);       
    }
    
    public void close() {

        if (myI2c != null) {
            myI2c.close();
            myI2c = null;
        }  
    }

    public boolean isAbsolute() {
        return true;
    }
}
