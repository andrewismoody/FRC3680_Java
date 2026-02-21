package frc.robot.positioner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.positioner.LimelightHelpers.PoseEstimate;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.proto.Photon;

public class PhotonVisionPositioner implements Positioner {

     public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    Transform3d kRobotToCam;
    PhotonPoseEstimator photonEstimator;
    PhotonCamera camera;
    String kCameraName = "photonvision";

    PhotonVisionPositioner(Transform3d robotToCam) {
        kRobotToCam = robotToCam;
    }

    public void Initialize() {
         photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
         camera = new PhotonCamera(kCameraName);
    }
    
    public Pose3d GetPose() {
        var poseEstimate = GetPoseEstimate();
        return poseEstimate != null ? poseEstimate : new Pose3d();
    }

    public void SetRobotOrientation(String name, double yaw, double yawRate, 
    double pitch, double pitchRate, 
    double roll, double rollRate) {
        
    }

    public boolean IsValid() {
        return true;
    }

    public boolean IsHealthy() {
        return true;
    }

    public String GetHealthReason() {
        return "not implemented";
    }

    public Pose3d GetReferenceInFieldCoords() {
        return GetPose().transformBy(kRobotToCam);
    }

    public Pose3d GetReferenceInRobotCoords() {
        return  GetPose().transformBy(kRobotToCam.inverse());
    }

    public Pose3d GetPoseEstimate() {
        var result = camera.getLatestResult();
        var visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
        if (visionEst.isEmpty()) {
            visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
        }

        return EstimatedPose.fromEstimatedRobotPose(visionEst.orElse(null));
    }
}