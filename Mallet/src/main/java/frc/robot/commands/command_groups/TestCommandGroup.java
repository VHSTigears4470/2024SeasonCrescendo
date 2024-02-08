import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TempAuto extends SequentialCommandGroup {
    public TempAuto(subsystems.....) {
        addRequirements(subsystem);
        addCommands(
            subsystem.setNoteOutputVoltageCommand,
            anothercommandreturningmethod
                   );
    }
}