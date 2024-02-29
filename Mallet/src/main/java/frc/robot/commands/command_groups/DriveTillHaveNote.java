package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.intake.IntakeSetIntakeVoltageEndWithBreakbeam;
import frc.robot.commands.drivebase.DriveForward;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveTillHaveNote extends ParallelRaceGroup {
    public DriveTillHaveNote(IntakeSubsystem intake, SwerveSubsystem swerve) {
        swerve.getHeading();
        addCommands(
                new IntakeSetIntakeVoltageEndWithBreakbeam(intake),
                new DriveForward(swerve, SwerveConstants.SWERVE_AUTO_VELOCITY));
    }
}
