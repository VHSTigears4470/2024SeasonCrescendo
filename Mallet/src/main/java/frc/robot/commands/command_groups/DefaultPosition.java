package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakePositionRetract;
import frc.robot.commands.intake.IntakePusherRetract;
import frc.robot.commands.intake.IntakeSetZeroVoltage;

public class DefaultPosition extends SequentialCommandGroup {
    public DefaultPosition(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
                // Delay retraction of intake to allow for elevator to go down first
                new ParallelCommandGroup(
                        new SequentialCommandGroup(new WaitCommand(.7), // TODO: Conditional to only wait for elevator
                                                                        // if its already in the up position
                                new IntakePositionRetract(intakeSubsystem)),
                        new ElevatorSetHeightState(elevatorSubsystem, ElevatorConstants.ELEVATOR_STATE.DOWN),
                        new IntakeSetZeroVoltage(intakeSubsystem),
                        new IntakePusherRetract(intakeSubsystem)));
    }
}