package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * A command group that shoots a note and then retracts the pusher.
 */
public class ShootSpeaker extends SequentialCommandGroup {
    public ShootSpeaker(IntakeSubsystem intakeSubsystem) {
        addRequirements(intakeSubsystem);
        addCommands(
                new IntakeSetNoteOutputVoltage(intakeSubsystem,
                new WaitCommand(0.5),
                new IntakeExtendPusher(intakeSubsystem),
                new WaitCommand(0.5),
                new IntakeSetZeroVoltage(intakeSubsystem),
                new IntakeRetractPusher(intakeSubsystem)));
    }
}