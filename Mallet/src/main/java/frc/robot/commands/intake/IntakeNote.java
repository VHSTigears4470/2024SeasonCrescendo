package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNote extends Command {
  private final IntakeSubsystem intake;

  public IntakeNote(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setIntakeVoltage();
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
    // When breakbeam detects note, stop the intake
    return intake.getNoteBreakbeam();
  }
}