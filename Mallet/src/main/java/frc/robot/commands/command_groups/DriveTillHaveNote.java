package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.drivebase.DriveForward;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveTillHaveNote extends SequentialCommandGroup {
    public DriveTillHaveNote(IntakeSubsystem intake, ElevatorSubsystem elevator, SwerveSubsystem swerve) {
        swerve.getHeading();
        addCommands(
            new IntakePosition(intake, elevator),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new IntakeNoteCommandGroup(intake, elevator),
                new DriveForward(swerve, SwerveConstants.SWERVE_AUTO_VELOCITY)
            ),
            new DefaultPosition(intake, elevator));
    }
}
