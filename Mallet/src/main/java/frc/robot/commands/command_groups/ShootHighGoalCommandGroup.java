package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakeExtendPusher;
import frc.robot.commands.intake.IntakeRetractPusher;
import frc.robot.commands.intake.IntakeSetAmpOutputVoltage;
import frc.robot.commands.intake.IntakeSetNoteOutputVoltage;
import frc.robot.commands.intake.IntakeSetZeroVoltage;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootHighGoalCommandGroup extends SequentialCommandGroup {
    public ShootHighGoalCommandGroup(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
                // Ignoring 1st & 2nd
                new ElevatorSetHeightState(elevatorSubsystem, ELEVATOR_STATE.UP),
                new IntakeSetAmpOutputVoltage(intakeSubsystem),
                new IntakeSetNoteOutputVoltage(intakeSubsystem,
                new WaitCommand(0.5),
                new IntakeExtendPusher(intakeSubsystem),
                new WaitCommand(0.5),
                new IntakeSetZeroVoltage(intakeSubsystem),
                new IntakeRetractPusher(intakeSubsystem)));
                
    }
}