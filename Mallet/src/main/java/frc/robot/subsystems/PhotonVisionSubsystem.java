package frc.robot.subsystems;

//Apriltag ambiguity threshhold

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

public class PhotonVisionSubsystem implements Runnable {
    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

    public PhotonVisionSubsystem(PhotonCamera cameraName, Transform3d robotToCamera) {
        this.photonCamera = cameraName;
        PhotonPoseEstimator photonPoseEstimator = null;
        try {
          var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
          // PV estimates will always be blue, they'll get flipped by robot thread
          layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
          if (photonCamera != null) {
            photonPoseEstimator = new PhotonPoseEstimator(
                layout, PoseStrategy.MULTI_TAG_PNP, photonCamera, robotToCamera);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
          }
        } catch (IOException e) {
          DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
          photonPoseEstimator = null;
        }
        this.photonPoseEstimator = photonPoseEstimator;
      }
}
