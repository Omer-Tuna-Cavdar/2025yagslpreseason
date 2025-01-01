package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.HelperMethodes.LimelightHelpers;
import java.util.Optional;

/**
 * Vision subsystem using a single Limelight camera for pose estimation and vision processing.
 */
public class Vision {

    private static final String LIMELIGHT_NAME = "limelight-front";
    private final Field2d field2d;

    /**
     * Constructor for the Vision class.
     *
     * @param field The field visualization object.
     */
    public Vision(Field2d field) {
        this.field2d = field;
    }

    /**
     * Updates the robot pose estimation using vision data from the Limelight.
     *
     * @param currentPose The current robot pose estimated by odometry.
     * @return The updated robot pose if valid data is available; otherwise, the current pose.
     */
    public Pose2d updatePoseEstimation(Pose2d currentPose) {
      LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
  
      if (poseEstimate != null && poseEstimate.tagCount > 0 && poseEstimate.latency < 100) { // Filter for valid data
          field2d.getObject("Vision Estimate").setPose(poseEstimate.pose);
          return poseEstimate.pose;
      }
  
      return currentPose;
  }
  

    /**
     * Gets the distance to a target using the Limelight's targeting capabilities.
     *
     * @return The distance to the target in meters, or -1 if no target is visible.
     */
    public double getDistanceToTarget() {
        if (LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            double ty = LimelightHelpers.getTY(LIMELIGHT_NAME);
            double targetHeightMeters = 2.5; // Example target height
            double limelightHeightMeters = 0.8; // Example Limelight height
            double limelightAngleDegrees = 30; // Example Limelight mounting angle

            return (targetHeightMeters - limelightHeightMeters)
                    / Math.tan(Math.toRadians(limelightAngleDegrees + ty));
        }

        return -1;
    }

    /**
     * Gets the target pose as estimated by the Limelight.
     *
     * @return The target pose in robot-relative space, or an empty optional if no target is visible.
     */
    public Optional<Pose3d> getTargetPose() {
        if (LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            return Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(LIMELIGHT_NAME));
        }
        return Optional.empty();
    }

    /**
     * Sets the Limelight's LED mode.
     *
     * @param enabled True to turn the LEDs on; false to turn them off.
     */
    public void setLEDMode(boolean enabled) {
        if (enabled) {
            LimelightHelpers.setLEDMode_ForceOn(LIMELIGHT_NAME);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_NAME);
        }
    }

    /**
     * Resets the field visualization.
     */
    public void resetField() {
        field2d.getObject("Vision Estimate").setPoses();
    }

    /**
     * Updates the field visualization with the tracked target pose.
     */
    public void updateFieldVisualization() {
        getTargetPose().ifPresent(targetPose ->
                field2d.getObject("Tracked Target").setPose(targetPose.toPose2d()));
    }
}
