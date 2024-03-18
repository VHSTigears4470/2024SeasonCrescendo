// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

  /** Swerve drive object. */
  private final SwerveDrive swerveDrive;

  /** Swerve pose estimator */
  // private final SwerveDrivePoseEstimator swervePoseEstimator;

  // Shuffleboard
  private ShuffleboardTab shuffleDebugTab;
  private GenericEntry entry_swerveHeading;

  // Photon Pose
  private Supplier<EstimatedRobotPose> rightPhotonPose;
  private Supplier<EstimatedRobotPose> leftPhotonPose;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    // Configure the Telemetry before creating the SwerveDrive to avoid
    // unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = SwerveConstants.TELEMETRY_VERBOSITY;
    if (RobotBase.isSimulation())
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {

      // TODO create gyro class for rotation 2D, set gyro angle based on path planner,
      // give it to estimator, get module positions from YAGSL, get initial Pose2D
      // from pathplanner
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED_METERS);
      // swervePoseEstimator = new SwerveDrivePoseEstimator(getKinematics(), );
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via
                                             // angle. Ex. Xbox Controller
    setupPathPlanner();
    initializeShuffleboard();
  }

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory          Directory of swerve drive config files.
   * @param limelightRobotPose Supplier for Robot Pose
   * @param limeightTimestamp  Supplier for Timestamp Pose was taken
   */
  public SwerveSubsystem(File directory, Supplier<Pose2d> rightPhotonPose, Supplier<Pose2d> leftPhotonPose) {
    // Configure the Telemetry before creating the SwerveDrive to avoid
    // unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = SwerveConstants.TELEMETRY_VERBOSITY;
    if (RobotBase.isSimulation())
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {

      // TODO create gyro class for rotation 2D, set gyro angle based on path planner,
      // give it to estimator, get module positions from YAGSL, get initial Pose2D
      // from pathplanner
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED_METERS);
      // swervePoseEstimator = new SwerveDrivePoseEstimator(getKinematics(), );
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via
                                             // angle. Ex. Xbox Controller
    setupPathPlanner();
    initializeShuffleboard();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, SwerveConstants.MAX_SPEED_METERS);
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0),
            // Translation PID constants
            new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                swerveDrive.swerveController.config.headingPIDF.i,
                swerveDrive.swerveController.config.headingPIDF.d),
            // Rotation PID constants
            4.5,
            // Max module speed, in m/s
            swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
            // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig()
        // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName       PathPlanner path name.
   * @param setOdomToStart Set the odometry position to the start of the path.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    if (setOdomToStart) {
      resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
    }

    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return AutoBuilder.followPath(path);
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true
   *                      to disable closed-loop.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    // Translation is where to go, rotation is speed to rotate, fieldRelative tells
    // whether the robot is measured based off the field or the robot, isOpenLoop is
    // the method to rotate (closed is constantly adjusting itself)
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumVelocity(), 4.0,
        swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother
   *                     controls.
   * @param translationY Translation in the Y direction. Cubed for smoother
   *                     controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getYaw().getRadians(),
          swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation     Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
          translationX.getAsDouble(),
          translationY.getAsDouble(),
          rotation.getAsDouble() * Math.PI,
          swerveDrive.getYaw().getRadians(),
          swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param RotationX    Rotation as a value between [-1, 1] converted to radians.
   * @param RotationY    Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier RotationX,
      DoubleSupplier RotationY) {
    swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
          translationX.getAsDouble(),
          translationY.getAsDouble(),
          RotationX.getAsDouble(),
          RotationY.getAsDouble(),
          swerveDrive.getYaw().getRadians(),
          swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Adds a vision measurement to the swerve
   * 
   * @param robotPose Pose of robot
   * @param timestamp Time the pose was taken in seconds
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp) {
    // Checks pose is not null
    if (robotPose != null) {
      swerveDrive.addVisionMeasurement(robotPose, timestamp);
    }
  }

  /**
   * Adds a vision measurement to the swerve
   * 
   * @param robotPose  Pose of robot
   * @param timestamp  Time the pose was taken in seconds
   * @param covariance The covariance of the measurement
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> covariance) {
    // Checks pose is not null
    if (robotPose != null) {
      swerveDrive.addVisionMeasurement(robotPose, timestamp, covariance);
    }
  }

  /**
   * Updates the vision measurement, should be called after setupVisionMeasurement
   * has been called
   * otherwise will not do anything
   */
  public void updateVisionMeasurement() {
    // Checks is the rightPhotonPose has been declared and has a pose to give
    if (rightPhotonPose != null && rightPhotonPose.get() != null) {
      addVisionMeasurement(rightPhotonPose.get().estimatedPose.toPose2d(), rightPhotonPose.get().timestampSeconds);
    }
    // Checks is the leftPhotonPose has been declared and has a pose to give
    if (leftPhotonPose != null && leftPhotonPose.get() != null) {
      addVisionMeasurement(leftPhotonPose.get().estimatedPose.toPose2d(), leftPhotonPose.get().timestampSeconds);
    }
  }

  /**
   * Sets up the vision measurement for swerve; after calling this function,
   * use updateVisionMeasurement, note addVisionMeasurement
   * 
   * @param robotPose Supplier for the pose of the robot
   * @param timestamp Supplier for the time the pose was taken in seconds
   */
  public void setupVisionMeasurement(Supplier<EstimatedRobotPose> leftPhotonPose,
      Supplier<EstimatedRobotPose> rightPhotonPose) {
    this.leftPhotonPose = leftPhotonPose;
    this.rightPhotonPose = rightPhotonPose;
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public void setPosition(Pose2d pose, Rotation3d rotation) {
    resetOdometry(pose);
    swerveDrive.setGyroOffset(rotation);
  }

  /**
   * Gets the SwerveDrivePoseEstimator
   * 
   * @return The SwerveDrivePoseEstimator
   */
  public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator() {
    return swerveDrive.swerveDrivePoseEstimator;
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW
   * positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        headingX,
        headingY,
        getHeading().getRadians(),
        SwerveConstants.MAX_SPEED_METERS);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        angle.getRadians(),
        getHeading().getRadians(),
        SwerveConstants.MAX_SPEED_METERS);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  private void initializeShuffleboard() {
    if (IntakeConstants.DEBUG) {
      shuffleDebugTab = Shuffleboard.getTab("Debug Tab");
      entry_swerveHeading = shuffleDebugTab.getLayout("Swerve", BuiltInLayouts.kList)
          .add("Swerve Heading", 0)
          .withWidget(BuiltInWidgets.kGyro)
          .getEntry();
    }
  }

  private void updateShuffleboard() {
    if (IntakeConstants.DEBUG) {
      // entry_swerveHeading.setValue(swerveDrive.getOdometryHeading());
    }
  }

  @Override
  public void periodic() {
    updateVisionMeasurement();
    updateShuffleboard();
  }

  @Override
  public void simulationPeriodic() {
  }
}
