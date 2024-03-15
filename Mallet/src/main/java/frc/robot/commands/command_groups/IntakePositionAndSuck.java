package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeSetIntakeVoltageEndWithBreakbeam;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePositionAndSuck extends SequentialCommandGroup {
    public IntakePositionAndSuck(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addRequirements(intakeSubsystem, elevatorSubsystem);
        // Start if there is no note in the intake
        if (!intakeSubsystem.getNoteBreakbeam()) {
            addCommands(
                    new ParallelCommandGroup(
                            new IntakePosition(intakeSubsystem, elevatorSubsystem),
                            new IntakeSetIntakeVoltageEndWithBreakbeam(intakeSubsystem)));
        }
    }
}