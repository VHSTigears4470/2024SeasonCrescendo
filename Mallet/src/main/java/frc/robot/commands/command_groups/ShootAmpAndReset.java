package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakePusherExtend;
import frc.robot.commands.intake.IntakePositionRetract;
import frc.robot.commands.intake.IntakeSetAmpVoltage;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootAmpAndReset extends SequentialCommandGroup {
    public ShootAmpAndReset(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        // TODO: precondition to do nothing if a note is not already in the intake
        addCommands(
                // TODO: Align to amp with on the fly path planning, ensure that the path is
                // actually only traversed when close enough to the amp to prevent collisions
                new IntakePositionRetract(intakeSubsystem),
                new ElevatorSetHeightState(elevatorSubsystem, ELEVATOR_STATE.UP),
                new IntakeSetAmpVoltage(intakeSubsystem),
                new WaitCommand(0.5),
                new IntakePusherExtend(intakeSubsystem),
                new WaitCommand(0.5),
                new DefaultPosition(intakeSubsystem, elevatorSubsystem));
    }
}