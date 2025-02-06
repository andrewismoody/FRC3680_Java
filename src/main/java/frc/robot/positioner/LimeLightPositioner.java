package frc.robot.positioner;

import edu.wpi.first.math.geometry.Translation3d;

public class LimeLightPositioner implements Positioner {
    public Translation3d GetPosition() {
        return LimelightHelpers.getBotPose3d("").getTranslation(); 
    }
}
