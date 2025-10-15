package frc.robot.modules;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.encoder.Encoder;

public class SwerveMotorDefinition {
  MotorController driveMotor;
  Encoder driveEncoder;

  MotorController rotatorMotor;
  Encoder angleEncoder;

    public SwerveMotorDefinition(MotorController driveMotor, Encoder driveEncoder, MotorController rotatorMotor, Encoder angleEncoder) {
        this.driveMotor = driveMotor;
        this.driveEncoder = driveEncoder;

        this.rotatorMotor = rotatorMotor;
        this.angleEncoder = angleEncoder;
    }
}
