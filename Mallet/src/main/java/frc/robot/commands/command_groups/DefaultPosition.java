package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakeRetractIntake;
import frc.robot.commands.intake.IntakeRetractPusher;
import frc.robot.commands.intake.IntakeSetZeroVoltage;

public class DefaultPosition extends SequentialCommandGroup {
    public DefaultPosition(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
                new ElevatorSetHeightState(elevatorSubsystem, ElevatorConstants.ELEVATOR_STATE.DOWN),
                new IntakeRetractIntake(intakeSubsystem),
                new IntakeSetZeroVoltage(intakeSubsystem),
                new IntakeRetractPusher(intakeSubsystem));
    }
}