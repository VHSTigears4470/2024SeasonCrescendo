// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteLimelight;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.Constants.SwerveConstants;

/**
 * Gets the offset and using robot's current heading, sets desired heading to
 * face the note
 */
public class FocusNote extends Command {

    private final SwerveSubsystem swerve;
    private final NoteLimelight limelight;
    private double headingError = Double.MAX_VALUE;

    /**
     * Creates a new Command.
     * Robot will rotate to face note
     * If the robot does not see a note, the command will finish
     * If the robot has been centerd, the command will finish
     * 
     * @param swerve The subsystem used by this command.
     */
    public FocusNote(SwerveSubsystem swerve, NoteLimelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Grabs the heading error from the network table and uses it to create a
        // direction to go to

        if (limelight.hasTarget()) {
            headingError = limelight.getXOffset();
        } else {
            headingError = 0;
        }

        // Get the desired chassis rotation in radians
        Rotation2d desiredHeading = swerve.getPose().getRotation().plus(Rotation2d.fromDegrees(headingError));
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(0, 0, desiredHeading);

        // Make the robot rotate only (no movement)
        swerve.drive(new Translation2d(0, 0), desiredSpeeds.omegaRadiansPerSecond, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stops robot
        swerve.drive(new Translation2d(0, 0), 0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Finishes once the note has been centered (if there is no note, ends)
        return !limelight.hasTarget() && Math.abs(headingError) < SwerveConstants.OFFSET_THRESHOLD;
    }
}