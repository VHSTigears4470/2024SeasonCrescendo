package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private double flywheelVoltage;
  private double feederVoltage;
  private double desiredVoltage;
  private double incrementVoltage;
  private double feederTime;
  private Timer timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(ShooterSubsystem shooterSubsystem, double desiredVoltage) {
    this.shooterSubsystem = shooterSubsystem;
    this.desiredVoltage = desiredVoltage;
    timer = new Timer();
    flywheelVoltage = 0;
    feederVoltage = 4;
    feederTime = 1;
    incrementVoltage = 0;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.moveFlywheel(desiredVoltage);
    timer.reset();
    timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(flywheelVoltage < desiredVoltage){
      flywheelVoltage += incrementVoltage;
    }else{
      shooterSubsystem.moveFeeder(feederVoltage);
      timer.start();
    }
    shooterSubsystem.moveFlywheel(flywheelVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheelVoltage = 0;
    shooterSubsystem.stop();
    timer.reset();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= feederTime;
  }
}
