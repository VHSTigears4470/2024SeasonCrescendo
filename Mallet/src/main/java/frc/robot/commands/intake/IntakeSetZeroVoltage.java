package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetZeroVoltage extends Command {
private final IntakeSubsystem intake;

  public IntakeSetZeroVoltage(IntakeSubsystem intake){
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setZeroVoltage();
  }

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}