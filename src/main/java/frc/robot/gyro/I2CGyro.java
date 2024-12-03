package frc.robot.gyro;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class I2CGyro extends GyroBase {
    private I2C myI2c;

    public I2CGyro(boolean UseFakeGyro) {
        this(Port.kOnboard, 0x68, 9, UseFakeGyro);
    }

    public I2CGyro(int readyPin, boolean UseFakeGyro) {
        this(Port.kOnboard, 0x68, readyPin, UseFakeGyro);
    }

    public I2CGyro(Port myPort, int readyPin, boolean UseFakeGyro) {
        this(myPort, 0x68, readyPin, UseFakeGyro);
    }

    public I2CGyro(Port myPort, int myAddr, int readyPin, boolean UseFakeGyro) {
        super(readyPin, UseFakeGyro);

        myI2c = new I2C(myPort, myAddr);
    }

    @Override
    public double[] getSensorData() {
        double[] sensorData = new double[6];

        // XYZ Accel
        sensorData[0] = readRegister(0x3B) / 16384.0;
        sensorData[1] = readRegister(0x3B + 2) / 16384.0;
        sensorData[2] = readRegister(0x3B + 4) / 16384.0;

        // XYZ Rate
        sensorData[3] = readRegister(0x43) / 131.0;
        sensorData[4] = readRegister(0x43 + 2) / 131.0;
        sensorData[5] = readRegister(0x43 + 4) / 131.0;

        return sensorData;
    }

    @Override
    public int readRegister(final int reg) {
        byte[] buf = new byte[2];

        myI2c.read(reg, 2, buf);

        return toUShort(buf);
    }
  
    @Override
    public void writeRegister(final int reg, final int val) {
        myI2c.write(reg, val);       
    }
    
    @Override
    public void close() {
        super.close();

        if (myI2c != null) {
            myI2c.close();
            myI2c = null;
        }  
    }
}
