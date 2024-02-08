package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.Constants.IntakeConstants.RETRACTED_INTAKE;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNoteCommandGroup extends SequentialCommandGroup {
    public IntakeNoteCommandGroup(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
                intakeSubsystem.retractPusherCommand(intakeSubsystem, RETRACTED_INTAKE.RETRACTED),
                new ElevatorSetHeightState(elevatorSubsystem, ELEVATOR_STATE.DOWN),
                intakeSubsystem.retractPusherCommand(intakeSubsystem, RETRACTED_INTAKE.RETRACTED));
    }
}