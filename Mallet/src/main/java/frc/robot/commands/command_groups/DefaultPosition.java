package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakePositionDown;
import frc.robot.commands.intake.IntakePositionUp;
import frc.robot.commands.intake.IntakePusherRetract;
import frc.robot.commands.intake.IntakeSetZeroVoltage;

public class DefaultPosition extends SequentialCommandGroup {
    public DefaultPosition(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addRequirements(intakeSubsystem, elevatorSubsystem);
        addCommands(
                // Delay retraction of intake to allow for elevator to go down first to prevent
                // out of bounds
                new IntakeSetZeroVoltage(intakeSubsystem), // Sequential because use same subsystem
                new IntakePusherRetract(intakeSubsystem),
                new IntakePositionDown(intakeSubsystem),
                new ConditionalCommand(new WaitCommand(
                        IntakeConstants.SECONDS_TILL_DOWN),
                        new WaitCommand(0),
                        () -> intakeSubsystem
                                .getIntakeState() != Value.kForward), // if
                                                                      // intake
                                                                      // not
                                                                      // down
                new ElevatorSetHeightState(elevatorSubsystem, ElevatorConstants.ELEVATOR_STATE.DOWN),
                new WaitUntilCommand(() -> elevatorSubsystem.isWithinPos(4)),
                new IntakePositionUp(intakeSubsystem),
                new WaitCommand(IntakeConstants.SECONDS_TILL_UP));
    }
}