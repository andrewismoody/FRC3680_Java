package frc.robot.positioner;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimeLightPositioner implements Positioner {
    boolean useMegatagTwo = true;

    public LimeLightPositioner(boolean UseMegatagTwo) {
        useMegatagTwo = UseMegatagTwo;
    }

    public void Initialize() {
        // // set IMU mode to 2 which fusions the provided orientation with the calculated positions
        // LimelightHelpers.SetIMUMode("", 2);

        // Make sure you only configure port forwarding once in your robot code.
        // Do not place these function calls in any periodic functions
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        };
    }

    public Translation3d GetPosition() {
        if (DriverStation.getAlliance().get() == Alliance.Red)
            return LimelightHelpers.getBotPose3d_wpiRed("").getTranslation();
        else
            return LimelightHelpers.getBotPose3d_wpiBlue("").getTranslation();
    }

    public void SetRobotOrientation(String limelightName, double yaw, double yawRate, 
    double pitch, double pitchRate, 
    double roll, double rollRate) {
        LimelightHelpers.SetRobotOrientation(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }

    public boolean IsValid() {
        //Always use blue alliance orientation according to 2024+ rules
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems

        if (useMegatagTwo) {
            if (DriverStation.getAlliance().get() == Alliance.Red)
                return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("").tagCount > 0;
            else 
                return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").tagCount > 0;
        } else {
            if (DriverStation.getAlliance().get() == Alliance.Red)
                return LimelightHelpers.getBotPoseEstimate_wpiRed("").tagCount > 0;
            else
                return LimelightHelpers.getBotPoseEstimate_wpiBlue("").tagCount > 0;
        }
    }
}
