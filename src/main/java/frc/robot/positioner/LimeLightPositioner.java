package frc.robot.positioner;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimeLightPositioner implements Positioner {
    boolean useMegatagTwo = true;

    public LimeLightPositioner(boolean UseMegatagTwo) {
        useMegatagTwo = UseMegatagTwo;
    }

    public Translation3d GetPosition() {
        return LimelightHelpers.getBotPose3d("").getTranslation(); 
    }

    public void SetRobotOrientation(String limelightName, double yaw, double yawRate, 
    double pitch, double pitchRate, 
    double roll, double rollRate) {
        LimelightHelpers.SetRobotOrientation(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }

    public boolean IsValid() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                if (useMegatagTwo) {
                    return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("").tagCount > 0;
                } else {
                    return LimelightHelpers.getBotPoseEstimate_wpiRed("").tagCount > 0;
                }
            } else {
                if (useMegatagTwo) {
                    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").tagCount > 0;
                } else {
                    return LimelightHelpers.getBotPoseEstimate_wpiBlue("").tagCount > 0;
                }
            }
        }

        return false;
    }
}
