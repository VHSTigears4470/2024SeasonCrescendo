package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakePositionDown;
import frc.robot.commands.intake.IntakePusherRetract;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePosition extends SequentialCommandGroup {
    public IntakePosition(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addRequirements(intakeSubsystem, elevatorSubsystem);
        addCommands(
                new IntakePusherRetract(intakeSubsystem),
                new IntakePositionDown(intakeSubsystem),
                new ElevatorSetHeightState(elevatorSubsystem, ELEVATOR_STATE.DOWN),
                new WaitCommand(IntakeConstants.SECONDS_TILL_DOWN),
                new WaitUntilCommand(elevatorSubsystem::isAtPos));
    }
}