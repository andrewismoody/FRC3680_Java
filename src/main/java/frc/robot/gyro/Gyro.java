package frc.robot.gyro;

public interface Gyro {

    public enum GyroAxis {
        /** The Gyro's X axis. */
        kX,
        /** The Gyro's Y axis. */
        kY,
        /** The Gyro's Z axis. */
        kZ
    }

  public void calibrate();

  public int setYawAxis(GyroAxis yaw_axis);

  public int readRegister(final int reg);

  public void writeRegister(final int reg, final int val);

  public void reset();

  public void acquire();

  public double getAngle();

  public double getRate();

  public GyroAxis getYawAxis();

  public double getGyroAngleX();

  public double getGyroAngleY();

  public double getGyroAngleZ();

  public double getGyroRateX();

  public double getGyroRateY();

  public double getGyroRateZ();

  public double getAccelX();

  public double getAccelY();

  public double getAccelZ();

  public double getXComplementaryAngle();

  public double getYComplementaryAngle();

  public double getXFilteredAccelAngle();

  public double getYFilteredAccelAngle();

  public void appendSimValueDeg(double angleDegrees);

}
