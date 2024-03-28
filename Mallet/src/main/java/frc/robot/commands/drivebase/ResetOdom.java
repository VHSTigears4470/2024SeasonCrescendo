// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.NoteLimelight;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class ResetOdom extends Command {

    private final SwerveSubsystem swerve;
    private boolean initRotation = false;
    private final double minOffset = 0.5; // TODO - change this
    private final double sensitivity = 0.5; // Sensetivity of the movement //TODO - Change this

    /**
     * Rotates to the note and drives forward - if there is no note, it will still
     * go forward but not rotate
     * Will never end unless forced to
     *
     * @param swerve    The swerve drivebase subsystem.
     * @param limelight The limelight subsystem
     */
    public ResetOdom(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.resetOdometry(new Pose2d());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerve.resetOdometry(new Pose2d());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}