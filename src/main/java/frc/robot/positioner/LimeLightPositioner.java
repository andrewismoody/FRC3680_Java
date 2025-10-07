package frc.robot.positioner;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimeLightPositioner implements Positioner {
    boolean useMegatagTwo = true;

    private Translation3d lastHealthPos = new Translation3d();
    private long lastHealthTsMs = 0L;
    private final double posJumpLimitMeters = 2.0; // treat larger jumps as invalid
    private final long posStaleTimeoutMs = 300;    // treat stale samples as invalid
    private long goodCount = 0;
    private long goodCountThreshold = 3;
    private long lastHealthCheckTs = 0L;
    private long lastValidCheckTs = 0L;
    private long lastPositionGetTs = 0L;

    boolean positionerHealthy = false;
    boolean positionValid = false;
    
    String healthReason = "none";

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
        long now = System.currentTimeMillis();

        if (Math.abs(now - lastPositionGetTs) < 15)
            return lastHealthPos; // don't check more than once per tick

        // only update position if we're healthy, otherwise return last good position
        if (positionerHealthy && goodCount >= goodCountThreshold) {
            if (DriverStation.getAlliance().get() == Alliance.Red)
                return LimelightHelpers.getBotPose3d_wpiRed("").getTranslation();
            else
                return LimelightHelpers.getBotPose3d_wpiBlue("").getTranslation();
        } else
            return lastHealthPos;
    }

    public void SetRobotOrientation(String limelightName, double yaw, double yawRate, 
    double pitch, double pitchRate, 
    double roll, double rollRate) {
        // TODO: re-evalutate this according to red/blue alliance positioning
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
            if (DriverStation.getAlliance().get() == Alliance.Red)
                valid = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("").tagCount > 0;
            else 
                valid = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").tagCount > 0;
        } else {
            if (DriverStation.getAlliance().get() == Alliance.Red)
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
        if (Math.abs(now - lastHealthCheckTs) < 15)
            return positionerHealthy; // don't check more than once per tick
        
        Translation3d pos = GetPosition();
        boolean bad = IsValid();
        boolean wasBad = false;

        if (!bad) {
            bad = Double.isNaN(pos.getX()) || Double.isNaN(pos.getY()) || Double.isNaN(pos.getZ()) ||
            Double.isInfinite(pos.getX()) || Double.isInfinite(pos.getY()) || Double.isInfinite(pos.getZ());
        } else {
            healthReason = "Not Valid";
            wasBad = true;
        }

        if (!bad) {
            if (pos != new Translation3d()) {
                double jump = lastHealthPos.minus(pos).getNorm();
                if (lastHealthTsMs > 0 && jump > posJumpLimitMeters) bad = true;
                if (lastHealthTsMs > 0 && (now - lastHealthTsMs) > posStaleTimeoutMs) bad = true;
            }
        } else if (!wasBad) {
            healthReason = "NAN/Infinite";
            wasBad = true;
        }

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

        lastHealthCheckTs = now;
        positionerHealthy = !bad;

        return positionerHealthy;
    }

    public String GetHealthReason() {
        return healthReason;
    }
}
