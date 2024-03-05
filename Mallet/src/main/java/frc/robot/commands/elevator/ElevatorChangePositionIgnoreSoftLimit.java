package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorChangePositionIgnoreSoftLimit extends Command {

  private final ElevatorSubsystem elevator;
  private final double incAmt;

  public ElevatorChangePositionIgnoreSoftLimit(ElevatorSubsystem elevator, double incAmt) {
    this.elevator = elevator;
    this.incAmt = incAmt;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.changePositionIgnoreSoftLimit(incAmt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
