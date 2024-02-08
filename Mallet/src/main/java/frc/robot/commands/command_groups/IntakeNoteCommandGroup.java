import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNoteCommandGroup extends SequentialCommandGroup {
    public IntakeNoteCommandGroup(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addCommands (
            intakeSubsystem.retractPusherCommand()
            elevatorSubsystem.setHeightStateCommand(DOWN);

        );
    
        
            intakeSubsystem.retractPusherCommand();
                   );
    }
}