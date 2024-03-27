package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakePositionDown;
import frc.robot.commands.intake.IntakePositionUp;

public class ClimbPosition extends SequentialCommandGroup {
    public ClimbPosition(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addRequirements(intakeSubsystem, elevatorSubsystem);
        addCommands(
                // TODO: Ask for more clarification on it needs to wait before intaking
                // (when it is getting ready to climb)
                new IntakePositionDown(intakeSubsystem),
                new ConditionalCommand(new WaitCommand(IntakeConstants.SECONDS_TILL_EXTEND),
                        new WaitCommand(0),
                        () -> intakeSubsystem.getIntakeState() != Value.kForward), // if intake not down
                new ElevatorSetHeightState(elevatorSubsystem,
                        ElevatorConstants.ELEVATOR_STATE.CLIMB),
                new WaitUntilCommand(() -> elevatorSubsystem.isWithinPos(4)),
                new IntakePositionUp(intakeSubsystem));
    }
}
