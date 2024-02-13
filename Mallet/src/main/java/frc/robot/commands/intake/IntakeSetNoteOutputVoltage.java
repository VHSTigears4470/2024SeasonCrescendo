package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetNoteOutputVoltage extends Command {
private final IntakeSubsystem intake;

  public IntakeSetNoteOutputVoltage(IntakeSubsystem intake, WaitCommand waitCommand, IntakeExtendPusher intakeExtendPusher, WaitCommand waitCommand2, IntakeSetZeroVoltage intakeSetZeroVoltage, IntakeRetractPusher intakeRetractPusher){
    this.intake = intake;
    addRequirements(intake);
  }

@Override
  public void initialize() {
    intake.setNoteOutputVoltage();
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