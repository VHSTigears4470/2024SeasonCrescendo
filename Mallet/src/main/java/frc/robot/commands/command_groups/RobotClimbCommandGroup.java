package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotClimbCommandGroup extends SequentialCommandGroup {
    public RobotClimbCommandGroup(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addRequirements(intakeSubsystem, elevatorSubsystem);
        addCommands( // TODO: Might not want a command group for this unless we have a timer between
                     // this
                new ClimbPosition(intakeSubsystem, elevatorSubsystem),
                new ElevatorSetHeightState(elevatorSubsystem, ELEVATOR_STATE.DOWN));
    }
}