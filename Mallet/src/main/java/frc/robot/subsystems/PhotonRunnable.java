package frc.robot.subsystems;

import static frc.robot.Constants.PhotonConstants.APRILTAG_AMBIGUITY_THRESHOLD;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;

public class PhotonRunnable implements Runnable {
    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

    public PhotonRunnable(PhotonCamera cameraName, Transform3d robotToCamera) {
      this.photonCamera = cameraName;
      PhotonPoseEstimator photonPoseEstimator = null;
      //try { *See if exception is needed
        var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        if (photonCamera != null) {
          photonPoseEstimator = new PhotonPoseEstimator(
              layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCamera);
          photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
      /* } catch (IOException e) {
        DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        photonPoseEstimator = null;
      }*/
      this.photonPoseEstimator = photonPoseEstimator;
    }
      
    @Override
    public void run() {
      // Get AprilTag data
      if (photonPoseEstimator != null && photonCamera != null) {
        var photonResults = photonCamera.getLatestResult();
        if (photonResults.hasTargets()
            && (photonResults.targets.size() > 1
                || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
          photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;
            // Make sure the measurement is on the field
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.FIELD_LENGTH_METERS
                && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FieldConstants.FIELD_WIDTH_METERS) {
              atomicEstimatedRobotPose.set(estimatedRobotPose);
            }
          });
        }
      }
    }
  
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }

}
