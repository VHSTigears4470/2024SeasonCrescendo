import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ElevatorConstants;

public class DefaultPosition extends SequentialCommandGroup {
    public DefaultPosition(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addRequirements(
                intakeSubsystem, elevatorSubsystem);
        addCommands(
                elevatorSubsystem.setHeightStateCommand(ElevatorConstants.ELEVATOR_STATE.DOWN),
                intakeSubsystem.retractIntakeCommand(),
                intakeSubsystem.setZeroVoltageCommand(),
                intakeSubsystem.retractPusher());
    }
}