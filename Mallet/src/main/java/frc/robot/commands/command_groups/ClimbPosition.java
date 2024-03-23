package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakePositionDown;

public class ClimbPosition extends SequentialCommandGroup {
    public ClimbPosition(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addRequirements(intakeSubsystem, elevatorSubsystem);
        addCommands(
                new ParallelCommandGroup(
                        // TODO: Ask for more clarification on it needs to wait before intaking
                        // (when it is getting ready to climb)
                        new IntakePositionDown(intakeSubsystem),
                        new ElevatorSetHeightState(elevatorSubsystem,
                                ElevatorConstants.ELEVATOR_STATE.CLIMB)));
    }
}
