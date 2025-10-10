package frc.robot.positioner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import frc.robot.misc.Utility;
import frc.robot.positioner.LimelightHelpers.PoseEstimate;

public class LimeLightPositioner implements Positioner {
    boolean useMegatagTwo = true;

    private Translation3d lastHealthPos = Translation3d.kZero;
    private long lastHealthTsMs = 0L;
    private final double posJumpLimitMeters = 2.0; // treat larger jumps as invalid
    private final long posStaleTimeoutMs = 300;    // treat stale samples as invalid
    private long goodCount = 0;
    private long goodCountThreshold = 3;
    // private long badCount = 0;
    // private long badCountThreshold = 3;
    // private long lastHealthCheckTs = 0L;
    private long lastValidCheckTs = 0L;
    // private long lastPositionGetTs = 0L;

    boolean positionerHealthy = false;
    boolean positionValid = false;
    
    String healthReason = "none";

    public LimeLightPositioner(boolean UseMegatagTwo) {
        useMegatagTwo = UseMegatagTwo;
    }

    public void Initialize() {
        // IMU mode to 2 only works with LL4 and fusions the provided orientation with the calculated positions
        LimelightHelpers.SetIMUMode("", 0);

        // Make sure you only configure port forwarding once in your robot code.
        // Do not place these function calls in any periodic functions
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        };
    }

    public Pose3d GetPose() {
        var poseEstimate = GetPoseEstimate();
        return new Pose3d(poseEstimate != null ? poseEstimate.pose : new Pose2d());
    }

    public PoseEstimate GetPoseEstimate() {
        PoseEstimate poseEstimate = null;
        // TODO 1: evaluate red/blue origin after adjusting coordinate systems
        if (useMegatagTwo) {
            if (Utility.IsRedAlliance())
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("");
            else
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        } else {
            if (Utility.IsRedAlliance())
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed("");
            else
                poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
        }

        return poseEstimate;
    }

    public Pose3d GetReferenceInFieldCoords() {
        Pose3d robotPose = GetPose();
        Pose3d cameraPose = LimelightHelpers.getCameraPose3d_RobotSpace("");
        // this looks backwards?
        Transform3d cameraTransform = new Transform3d(cameraPose.getTranslation(), cameraPose.getRotation());
        cameraPose = robotPose.transformBy(cameraTransform);

        return cameraPose;
    }

    public Pose3d GetReferenceInRobotCoords() {
        Pose3d cameraPose = LimelightHelpers.getCameraPose3d_RobotSpace("");

        return cameraPose;
    }

    // SetRobotOrientation - yaw is in degrees
    public void SetRobotOrientation(String limelightName, double yaw, double yawRate, 
    double pitch, double pitchRate, 
    double roll, double rollRate) {
        // TODO 1: evaluate red/blue origin after adjusting coordinate systems
        if (Utility.IsRedAlliance())
            yaw = (yaw + 180) % 360;

        LimelightHelpers.SetRobotOrientation(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }

    public boolean IsValid() {
        //Always use blue alliance orientation according to 2024+ rules
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems
        long now = System.currentTimeMillis();

        if (Math.abs(now - lastValidCheckTs) < 15)
            return positionValid; // don't check more than once per tick

        boolean valid = false;
        
        if (useMegatagTwo) {
            if (Utility.IsRedAlliance())
                valid = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("").tagCount > 0;
            else 
                valid = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").tagCount > 0;
        } else {
            if (Utility.IsRedAlliance())
                valid = LimelightHelpers.getBotPoseEstimate_wpiRed("").tagCount > 0;
            else
                valid = LimelightHelpers.getBotPoseEstimate_wpiBlue("").tagCount > 0;
        }

        positionValid = valid;
        lastValidCheckTs = now;

        return positionValid;
    }

    public boolean IsHealthy() {
        long now = System.currentTimeMillis();
        // if (Math.abs(now - lastHealthCheckTs) < 15)
        //     return positionerHealthy; // don't check more than once per tick
        
        Translation3d pos = GetPose().getTranslation();
        boolean bad = false;
        boolean wasBad = false;

        //bad = IsValid();
        // if (!bad) {
        //     bad = Double.isNaN(pos.getX()) || Double.isNaN(pos.getY()) || Double.isNaN(pos.getZ()) ||
        //     Double.isInfinite(pos.getX()) || Double.isInfinite(pos.getY()) || Double.isInfinite(pos.getZ());
        // } else {
        //     healthReason = "Not Valid";
        //     wasBad = true;
        // }

        // if (!bad) {
        //     // don't check for jump if we haven't found our position yet
        //     if (pos.getNorm()> 0.0 && lastHealthPos.getNorm() > 0.0) {
        //         double jump = lastHealthPos.minus(pos).getNorm();
        //         if (lastHealthTsMs > 0 && jump > posJumpLimitMeters) bad = true;
        //         if (lastHealthTsMs > 0 && (now - lastHealthTsMs) > posStaleTimeoutMs) bad = true;
        //     }
        // } else if (!wasBad) {
        //     healthReason = "NAN/Infinite";
        //     wasBad = true;
        // }

        if (!bad) {
            lastHealthPos = pos;
            lastHealthTsMs = now;
            // wait for a few samples to ensure that we're stable before we start sending position updates
            if (goodCount < goodCountThreshold) {
                goodCount++;
                healthReason = "warming";
            }
            else
                healthReason = "ok";
        } else {
            // all bad conditions flow here
            goodCount = 0;
            if (!wasBad)
                healthReason = "jump";
        }

        // lastHealthCheckTs = now;
        positionerHealthy = !bad;

        return positionerHealthy;
    }

    public String GetHealthReason() {
        return healthReason;
    }
}
