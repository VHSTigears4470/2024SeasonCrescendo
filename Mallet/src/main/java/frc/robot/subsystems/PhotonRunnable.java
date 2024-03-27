package frc.robot.subsystems;

import static frc.robot.Constants.PhotonConstants.APRILTAG_AMBIGUITY_THRESHOLD;

import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.PhotonConstants;

public class PhotonRunnable implements Runnable {
  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  // Atomic Reference is reportedly used for atomic operations (aka thread-safe)
  // and affect the reference to the object, not a copy
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

  public PhotonRunnable(PhotonCamera cameraName, Transform3d robotToCamera) {
    this.photonCamera = cameraName;
    PhotonPoseEstimator photonPoseEstimator = null;
    try {
      // Sets photon to estimate based on correct field side
      AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      // Should not change as all other subsystems rely on blue alliance
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      if (photonCamera != null) {
        photonPoseEstimator = new PhotonPoseEstimator(
            layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCamera);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      }
    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout",
          e.getStackTrace());
      photonPoseEstimator = null;
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null) {

      PhotonPipelineResult photonResults = photonCamera.getLatestResult();
      // Checks if photon can see more than 2 tags or the 1st tag can be seen well
      if (photonResults.hasTargets() && (photonResults.targets.size() > 1 ||
          photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
        // Updates if can get data, estimatedRobotPose is said data pose
        photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
          Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
          // Make sure the measurement is on the field
          if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= PhotonConstants.FIELD_LENGTH_METERS
              && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= PhotonConstants.FIELD_WIDTH_METERS) {
            atomicEstimatedRobotPose.set(estimatedRobotPose);
          }
        });
      }
    }
  }

  /**
   * Gets Estimated Pose
   * 
   * @return gets the latest pose reported by the camera
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }

}
