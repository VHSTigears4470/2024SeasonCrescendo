// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteLimelight;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import frc.robot.Constants.SwerveConstants;

/**
 * An example command that uses an example subsystem.
 */
public class CenterToFaceNote extends Command {

    private final SwerveSubsystem swerve;
    private final NoteLimelight limelight;

    private final double offsetThreshold = 0.5; //TODO - change value
    private final double omegaRadiansPerSecond = 20 / Math.PI; //TODO - change value

    /**
     * Creates a new Command.
     * Robot will rotate to face note
     * If the robot does not see a note, the command will finish
     * If the robot has been centerd, the command will finish
     * @param swerve The subsystem used by this command.
     */
    public CenterToFaceNote(SwerveSubsystem swerve, NoteLimelight limelight) {
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
        if(limelight.hasTarget()) {
            // Keep rotating robot (direction depends on the x offset, aka if it is to the left or right)
            //TODO - Check if offset direction is correct
            swerve.drive(new Translation2d(0, 0), Math.signum(limelight.getXOffset()) * omegaRadiansPerSecond, true);
        }
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
        return limelight.hasTarget() && Math.abs(limelight.getXOffset()) < offsetThreshold;
    }
}