// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.NoteLimelight;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class DriveForwardWithFocus extends Command {

  private final SwerveSubsystem swerve;
  private final NoteLimelight limelight;
  private boolean initRotation = false;
  private final double maxOffset = 20; //TODO - cahnge this
  private final double minOffset = 0.5; //TODO - change this
  private final double sensitivity = 0.5; // Sensetivity of the movement //TODO - Change this

  /**
   * Rotates to the note and drives forward - if there is no note, it will still go forward but not rotate
   * Will never end unless forced to
   *
   * @param swerve            The swerve drivebase subsystem.
   * @param limelight         The limelight subsystem
   */
  public DriveForwardWithFocus(SwerveSubsystem swerve, NoteLimelight limelight) {
    this.swerve = swerve;
    this.limelight = limelight;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    initRotation = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double calcOffset;
    // Checks whether the robot has a note to rotate to
    if(limelight.hasTarget()){
      // Calculates the offset / speed to rotate to. Has a max of speed of maxOffset
      calcOffset = Math.signum(limelight.getXOffset()) * Math.min(Math.abs(limelight.getXOffset()), maxOffset);
    } else {
      calcOffset = 0;
    }
    // Drives forward at a certain rotation
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(
        swerve.getPose().getRotation().getCos() * sensitivity, 
        swerve.getPose().getRotation().getSin() * sensitivity, 
        new Rotation2d(calcOffset)
    );

    // Prevent Movement After Auto
    if (initRotation) {
      if (calcOffset < minOffset) {
        // Get the curretHeading
        Rotation2d firstLoopHeading = swerve.getHeading();

        // Set the Current Heading to the desired Heading
        desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
      }
      // Dont Init Rotation Again
      initRotation = false;
    }

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        SwerveConstants.LOOP_TIME, SwerveConstants.ROBOT_MASS, List.of(SwerveConstants.CHASSIS),
        swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the robot
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}