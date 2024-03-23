package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
                new ParallelCommandGroup(
                        new IntakePusherRetract(intakeSubsystem),
                        new IntakePositionDown(intakeSubsystem),
                        new ElevatorSetHeightState(elevatorSubsystem, ELEVATOR_STATE.DOWN)));
    }
}