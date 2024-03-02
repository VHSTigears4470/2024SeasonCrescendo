package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeSetIntakeVoltageEndWithBreakbeam;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePositionAndSuck extends SequentialCommandGroup {
    public IntakePositionAndSuck(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        // TODO: precondition to do nothing if a note is already in the intake
        addCommands(
                new ParallelCommandGroup(
                        new IntakePosition(intakeSubsystem, elevatorSubsystem),
                        new IntakeSetIntakeVoltageEndWithBreakbeam(intakeSubsystem)));
    }
}