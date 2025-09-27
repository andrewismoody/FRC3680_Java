package frc.robot.encoder;

public interface Encoder {
    public double getDistance();
    public double getRawValue();
    public void setDistancePerPulse(double dpp);
    public void setReverseDirection(boolean reverse);
    public void setAngleOffsetDeg(double value);
    public void setAngleOffsetRad(double value);
    public boolean isAbsolute();
    public void setZeroPosition();
    public double getAngleOffsetRad();
    public void setMultiplier(double value);
    public double getMultiplier();
}
