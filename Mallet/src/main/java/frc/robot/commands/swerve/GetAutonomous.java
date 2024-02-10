package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class GetAutonomous extends Command {
    private final SwerveSubsystem swerve;
    private final String pathName;
    private final boolean setOdomToStart;

    public GetAutonomous(SwerveSubsystem swerve, double incAmt) {
    this.elevator = elevator;
    this.incAmt = incAmt;
    addRequirements(elevator);
  }
}