package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.intake.IntakePusherExtend;
import frc.robot.Constants.CycleTimes;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakePositionUp;
import frc.robot.commands.intake.IntakeSetSpeakerVoltage;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootSpeakerAndReset extends SequentialCommandGroup {
    public ShootSpeakerAndReset(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addRequirements(intakeSubsystem, elevatorSubsystem);
        // Start if note is in the intake
        addCommands(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ElevatorSetHeightState(elevatorSubsystem, ELEVATOR_STATE.DOWN),
                                new WaitUntilCommand(elevatorSubsystem::isAtPos), // wait until it is at position
                                new IntakePositionUp(intakeSubsystem),
                                new ConditionalCommand(new WaitCommand(IntakeConstants.SECONDS_TILL_RETRACT),
                                        new WaitCommand(0),
                                        () -> intakeSubsystem.getIntakeState() != Value.kReverse), // if intake not up
                                new IntakeSetSpeakerVoltage(intakeSubsystem),
                                new WaitCommand(CycleTimes.INTAKE_MOTORS_WARM_UP_CYCLE_TIME),
                                new IntakePusherExtend(intakeSubsystem),
                                new WaitCommand(CycleTimes.INTAKE_MOTORS_WARM_UP_CYCLE_TIME),
                                new DefaultPosition(intakeSubsystem, elevatorSubsystem)),
                        new WaitCommand(0),
                        intakeSubsystem::noteBreambeamTripped)
        // TODO: Light on the robot glows when in correct range/radius around the
        // speaker and pointing in correct direction
        );

    }
}