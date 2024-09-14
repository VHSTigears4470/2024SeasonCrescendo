package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.drivebase.DriveForward;
import frc.robot.commands.intake.IntakeSetIntakeVoltageEndWithBreakbeam;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveTillHaveNote extends SequentialCommandGroup {
    public DriveTillHaveNote(IntakeSubsystem intake, ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerve) {
        addRequirements(intake, elevatorSubsystem, swerve);
        addCommands(
                new WaitUntilCommand(elevatorSubsystem::isAtPos),
                new ParallelRaceGroup(
                        new IntakeSetIntakeVoltageEndWithBreakbeam(intake),
                        new DriveForward(swerve, SwerveConstants.SWERVE_AUTO_VELOCITY),
                        new WaitCommand(1.5)) // time out so don't keep driving
        );
    }
}
