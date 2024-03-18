package frc.robot.subsystems;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
//import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.KRedAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class PoseEstimation extends SubsystemBase {
  private final Field2d field2d = new Field2d();
  private final PhotonRunnable rightEstimator = new PhotonRunnable(new PhotonCamera("rightCamera"),
      Constants.PhotonConstants.ROBOT_TO_RIGHT_PHOTON);
  private final PhotonRunnable leftEstimator = new PhotonRunnable(new PhotonCamera("leftCamera"),
      Constants.PhotonConstants.ROBOT_TO_LEFT_PHOTON);

  private final Notifier allNotifier = new Notifier(() -> {
    rightEstimator.run();
    leftEstimator.run();
  });

  private OriginPosition originPosition = kBlueAllianceWallRightSide;
  private SwerveSubsystem swerveSub;

  public PoseEstimation(SwerveSubsystem swerveSub) {
    this.swerveSub = swerveSub;

    // Start PhotonVision thread
    allNotifier.setName("runAll");
    allNotifier.startPeriodic(0.02);
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
      estimatorChecker(rightEstimator);
      estimatorChecker(leftEstimator);
    } else {
      allNotifier.close();
    }

    // estimatorChecker(backEstimator);

    // Set the pose on the dashboard
    var dashboardPose = swerveSub.getPose();
    if (originPosition == kRedAllianceWallRightSide) {
      // Flip the pose when red, since the dashboard field photo cannot be rotated
      dashboardPose = flipAlliance(dashboardPose);
    }
    field2d.setRobotPose(dashboardPose);
  }

  private String getFormattedPose() {
    var pose = swerveSub.getPose();
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

  private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
    double smallestDistance = Double.POSITIVE_INFINITY;
    for (var target : estimation.targetsUsed) {
      var t3d = target.getBestCameraToTarget();
      var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance)
        smallestDistance = distance;
    }
    double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
        ? 1
        : Math.max(
            1,
            (estimation.targetsUsed.get(0).getPoseAmbiguity()
                + Constants.PhotonConstants.POSE_AMBIGUITY_SHIFTER)
                * Constants.PhotonConstants.POSE_AMBIGUITY_MULTIPLIER);
    double confidenceMultiplier = Math.max(
        1,
        (Math.max(
            1,
            Math.max(0, smallestDistance - Constants.PhotonConstants.NOISY_DISTANCE_METERS)
                * Constants.PhotonConstants.DISTANCE_WEIGHT)
            * poseAmbiguityFactor)
            / (1
                + ((estimation.targetsUsed.size() - 1) * Constants.PhotonConstants.TAG_PRESENCE_WEIGHT)));

    return Constants.PhotonConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
  }

  public void estimatorChecker(PhotonRunnable estimator) {
    var cameraPose = estimator.grabLatestEstimatedPose();
    if (cameraPose != null) {
      // New pose from vision
      var pose2d = cameraPose.estimatedPose.toPose2d();
      if (originPosition == kRedAllianceWallRightSide) {
        pose2d = flipAlliance(pose2d);
      }
      swerveSub.addVisionMeasurement(pose2d, cameraPose.timestampSeconds,
          confidenceCalculator(cameraPose));
    }
  }
}
