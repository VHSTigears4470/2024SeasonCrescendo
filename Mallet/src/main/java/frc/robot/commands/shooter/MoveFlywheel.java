package frc.robot.commands.shooter;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveFlywheel extends Command {
  private final ShooterSubsystem shooterSub;
  private final double voltage;

  /**
   * Creates a new MoveFlywheel Command.
   *
   * @param shooter The shooter subsystem.
   * @param vol     The voltage at which to move the flywheel
   */
  public MoveFlywheel(ShooterSubsystem shooter, double vol) {
    shooterSub = shooter;
    voltage = vol;

    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSub.moveFlywheel(voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.moveFlywheel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
