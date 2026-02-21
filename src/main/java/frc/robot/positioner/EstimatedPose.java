package frc.robot.positioner;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.positioner.LimelightHelpers.PoseEstimate;

public class EstimatedPose {
    public static Pose3d fromPoseEstimate(PoseEstimate poseEstimate) {
        return new Pose3d(poseEstimate.pose);
    }

    public static Pose3d fromEstimatedRobotPose(EstimatedRobotPose estimatedRobotPose) {
        return estimatedRobotPose.estimatedPose;
    }
}
