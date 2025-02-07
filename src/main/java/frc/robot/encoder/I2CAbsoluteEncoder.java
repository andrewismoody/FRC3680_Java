package frc.robot.encoder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class I2CAbsoluteEncoder implements Encoder {
    private I2C myI2c;
    double angleOffsetRad = 0.0; // angle to subtract from actual angle to zero the encoder

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

    public void setAngleOffsetDeg(double value) {
        setAngleOffsetRad(value * 0.0174532);
    }

    public void setAngleOffsetRad(double value) {
        angleOffsetRad = value;
    }

    public double getRawValue() {
        return readRegister(0x0C);                        // combine bytes to get 12-bit value 11:0
    }

    public double getDistance() {
        double rawAngle;

        rawAngle = getRawValue();

        boolean reportRadians = false;

        // TODO: implement angle offset
        
        if (reportRadians)
            return rawAngle * 0.001533203125; // 6.28 / 4096 = 0.001533203125 (radians)
        else
            return rawAngle * 0.087890625; // or 360/4096 = 0.087890625 (degrees)

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
}
