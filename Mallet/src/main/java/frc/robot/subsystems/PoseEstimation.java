package frc.robot.subsystems;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
//import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.KRedAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PhotonConstants;

public class PoseEstimation extends SubsystemBase {
  private final Field2d field2d = new Field2d();
  private final PhotonRunnable rightEstimator;
  private final PhotonRunnable leftEstimator;

  private final Notifier allNotifier;

  // Where the robot reports (0, 0) (aka the origin)
  private OriginPosition originPosition = kBlueAllianceWallRightSide;
  private SwerveSubsystem swerveSub;

  public PoseEstimation(SwerveSubsystem swerveSub) {
    this.swerveSub = swerveSub;

    if(PhotonConstants.USING_RIGHT_PHOTON) {
        rightEstimator = new PhotonRunnable(new PhotonCamera(PhotonConstants.RIGHT_PHOTON_NAME), 
            Constants.PhotonConstants.ROBOT_TO_RIGHT_PHOTON);
    } else {
        rightEstimator = null;
    }

    if(PhotonConstants.USING_LEFT_PHOTON) {
        leftEstimator =  new PhotonRunnable(new PhotonCamera(PhotonConstants.LEFT_PHOTON_NAME),
            Constants.PhotonConstants.ROBOT_TO_LEFT_PHOTON);
    } else {
        leftEstimator = null;
    }
    
    allNotifier =  new Notifier(() -> {
        if(PhotonConstants.USING_RIGHT_PHOTON) {
            rightEstimator.run();
        }
        if(PhotonConstants.USING_LEFT_PHOTON) {
            leftEstimator.run();
        }
    });

    // Start PhotonVision thread
    allNotifier.setName("runAll");
    allNotifier.startPeriodic(0.2);
  }

  public void addDashboardWidgets(ShuffleboardTab tab) {
    tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
    tab.addString("Pose", this::getFormattedPose).withPosition(6, 2).withSize(2, 1);
  }

  /**
   * Sets the alliance. This is used to configure the origin of the AprilTag map
   * 
   * @param alliance alliance
   */
  public void setAlliance(Alliance alliance) {
    boolean allianceChanged = false;
    switch (alliance) {
      case Blue:
        allianceChanged = (originPosition == kRedAllianceWallRightSide);
        originPosition = kBlueAllianceWallRightSide;
        break;
      case Red:
        allianceChanged = (originPosition == kBlueAllianceWallRightSide);
        originPosition = kRedAllianceWallRightSide;
        break;
      default:
        // No valid alliance data. Nothing we can do about it
        break;
    }

    if (allianceChanged) {
      // The alliance changed, which changes the coordinate system.
      // Since a tag was seen, and the tags are all relative to the coordinate system,
      // the estimated pose
      // needs to be transformed to the new coordinate system.
      var newPose = flipAlliance(swerveSub.getPose());
      swerveSub.resetOdometry(newPose);
    }
  }

  @Override
  public void periodic() {
    if (Constants.PhotonConstants.USING_VISION) {
      // Adds both camera reported pose to swerve
      if(PhotonConstants.USING_RIGHT_PHOTON) {
        estimatorChecker(rightEstimator);
      }
      if(PhotonConstants.USING_LEFT_PHOTON) {
        estimatorChecker(leftEstimator);
      }
    } else {
      allNotifier.close();
    }

    // estimatorChecker(backEstimator);

    // Set the pose on the dashboard
    Pose2d dashboardPose = swerveSub.getPose();
    if (originPosition == kRedAllianceWallRightSide) {
      // Flip the pose when red, since the dashboard field photo cannot be rotated
      dashboardPose = flipAlliance(dashboardPose);
    }
    field2d.setRobotPose(dashboardPose);
  }

  /**
   * Reterns the pose
   * Sample Output (X.XX, Y.YY) D.D degrees
   * D is degrees
   * @return Returns X, Y, and Degrees 
   */
  private String getFormattedPose() {
    Pose2d pose = swerveSub.getPose();
    return String.format("(%.3f, %.3f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    swerveSub.resetOdometry(new Pose2d());
  }

  /**
   * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
   * always on the right corner of your
   * alliance wall, so for 2023, the field elements are at different coordinates
   * for each alliance.
   * 
   * @param poseToFlip pose to transform to the other alliance
   * @return pose relative to the other alliance's coordinate system
   */
  private Pose2d flipAlliance(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(new Pose2d(
        new Translation2d(Constants.PhotonConstants.FIELD_LENGTH_METERS, Constants.PhotonConstants.FIELD_WIDTH_METERS),
        new Rotation2d(Math.PI)));
  }

  public void addTrajectory(Trajectory traj) {
    field2d.getObject("Trajectory").setTrajectory(traj);
  }

  // public void resetPoseRating() {
  // xValues.clear();
  // yValues.clear();
  // }

  /**
   * 
   * @param estimation Camera to get data from
   * @return A matrix of 3 by 1 that represents the confidence for x, y, and theta in radians
   */
  private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
    // Defaults to very untrustworthy
    double smallestDistance = Double.POSITIVE_INFINITY;

    // Looks at each AprilTag used to calculate pose and determine the confidence
    // Sets distance to best one (aka closest)
    for (PhotonTrackedTarget target : estimation.targetsUsed) {
      // Distance in 3d space from camera to april tag
      Transform3d t3d = target.getBestCameraToTarget();
      // Calculates actual linear distance from camera to target
      double distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance)
        smallestDistance = distance;
    }
    // If number of tags is not one, then set factor multiplier to one
    // Otherwise, calculate ambiguity factor 
    double poseAmbiguityFactor = estimation.targetsUsed.size() != 1 ? 1 : Math.max(
            1,
            (estimation.targetsUsed.get(0).getPoseAmbiguity()
                + Constants.PhotonConstants.POSE_AMBIGUITY_SHIFTER)
                * Constants.PhotonConstants.POSE_AMBIGUITY_MULTIPLIER);
    double confidenceMultiplier = Math.max(
        1,
        (
            Math.max(
                1,
                Math.max(
                    0, smallestDistance - Constants.PhotonConstants.NOISY_DISTANCE_METERS
                ) * Constants.PhotonConstants.DISTANCE_WEIGHT
            )* poseAmbiguityFactor
        )/ (1 + ((estimation.targetsUsed.size() - 1) * Constants.PhotonConstants.TAG_PRESENCE_WEIGHT))
      );
    
    return Constants.PhotonConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
  }

  /**
   * If possible, adds vision meausrement to swerve
   * @param estimator Camera to get pose from
   */
  public void estimatorChecker(PhotonRunnable estimator) {
    EstimatedRobotPose cameraPose = estimator.grabLatestEstimatedPose();
    // Check if there is actually a pose
    if (cameraPose != null) {
      // New pose from vision
      var pose2d = cameraPose.estimatedPose.toPose2d();
      // Orientates to right coords
      if (originPosition == kRedAllianceWallRightSide) {
        pose2d = flipAlliance(pose2d);
      }
      // Adds vision to swerve
      swerveSub.addVisionMeasurement(pose2d, cameraPose.timestampSeconds,
          confidenceCalculator(cameraPose));
    }
  }
}
