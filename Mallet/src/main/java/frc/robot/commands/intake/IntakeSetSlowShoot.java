package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Sets intakes spinning at voltage for sucking in notes.
 * Ends when the note is detected by the breakbeam.
 */
public class IntakeSetSlowShoot extends Command {
  private final IntakeSubsystem intake;

  public IntakeSetSlowShoot(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setSlowOutputVoltage();
  }

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setZeroVoltage();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;  
  }
}