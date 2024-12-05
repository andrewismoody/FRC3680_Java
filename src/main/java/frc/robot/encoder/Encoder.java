package frc.robot.encoder;

public interface Encoder {

    public double getDistance();
    public void setDistancePerPulse(double dpp);
    public void setReverseDirection(boolean reverse);
}
