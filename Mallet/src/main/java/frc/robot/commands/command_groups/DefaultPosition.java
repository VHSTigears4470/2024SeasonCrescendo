package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakePositionUp;
import frc.robot.commands.intake.IntakePusherRetract;
import frc.robot.commands.intake.IntakeSetZeroVoltage;

public class DefaultPosition extends SequentialCommandGroup {
    public DefaultPosition(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addRequirements(intakeSubsystem, elevatorSubsystem);
        addCommands(
                // Delay retraction of intake to allow for elevator to go down first
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ElevatorSetHeightState(elevatorSubsystem, ElevatorConstants.ELEVATOR_STATE.DOWN),
                                new WaitUntilCommand(elevatorSubsystem::isAtPos),
                                new IntakePositionUp(intakeSubsystem)),
                        new SequentialCommandGroup(
                                new IntakeSetZeroVoltage(intakeSubsystem), // Sequential because use same subsystem
                                new IntakePusherRetract(intakeSubsystem))));
    }
}