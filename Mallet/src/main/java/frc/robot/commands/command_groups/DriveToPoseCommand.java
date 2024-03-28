package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPoseCommand extends SequentialCommandGroup {
    public DriveToPoseCommand(SwerveSubsystem swerveSub) {
        addRequirements(swerveSub);
        addCommands(
                swerveSub.driveToPose(0, 0));
    }
}
