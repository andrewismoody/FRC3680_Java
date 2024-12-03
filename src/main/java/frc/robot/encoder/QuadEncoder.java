package frc.robot.encoder;

public class QuadEncoder implements Encoder {
    
    edu.wpi.first.wpilibj.Encoder internalEncoder;

    public QuadEncoder(int pinA, int pinB) {
        internalEncoder = new edu.wpi.first.wpilibj.Encoder(pinA, pinB);

        internalEncoder.reset();
    }

    @Override
    public void setDistancePerPulse(double dpp) {
        internalEncoder.setDistancePerPulse(dpp);
    }

    @Override
    public void setReverseDirection(boolean reverse) {
        internalEncoder.setReverseDirection(reverse);
    }

    @Override
    public double getDistance() {
        return internalEncoder.getDistance();
    }
}
