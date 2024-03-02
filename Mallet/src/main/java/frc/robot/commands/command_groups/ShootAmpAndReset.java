package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CycleTimes;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakePusherExtend;
import frc.robot.commands.intake.IntakePositionRetract;
import frc.robot.commands.intake.IntakeSetAmpVoltage;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootAmpAndReset extends SequentialCommandGroup {
    public ShootAmpAndReset(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        // Start if note is in the intake
        if(intakeSubsystem.getNoteBreakbeam())
        {
            addCommands(
                    // TODO: Align to amp with on the fly path planning, ensure that the path is
                    // actually only traversed when close enough to the amp to prevent collisions
                    new IntakePositionRetract(intakeSubsystem),
                    new ElevatorSetHeightState(elevatorSubsystem, ELEVATOR_STATE.UP),
                    new IntakeSetAmpVoltage(intakeSubsystem),
                    new WaitCommand(CycleTimes.INTAKE_MOTORS_WARM_UP_CYCLE_TIME),
                    new IntakePusherExtend(intakeSubsystem),
                    new WaitCommand(CycleTimes.INTAKE_MOTORS_WARM_UP_CYCLE_TIME),
                    new DefaultPosition(intakeSubsystem, elevatorSubsystem));
        }
    }
}