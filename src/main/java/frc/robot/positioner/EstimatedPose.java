package frc.robot.positioner;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.positioner.LimelightHelpers.PoseEstimate;

public class EstimatedPose {
    public static Pose3d fromPoseEstimate(PoseEstimate poseEstimate) {
        if (poseEstimate == null)
            return null;

        return new Pose3d(poseEstimate.pose);
    }

    public static Pose3d fromEstimatedRobotPose(EstimatedRobotPose estimatedRobotPose) {
        if (estimatedRobotPose == null)
            return null;
            
        return estimatedRobotPose.estimatedPose;
    }
}
