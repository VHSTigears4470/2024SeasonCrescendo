package frc.robot.commands.autos;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroup_LeaveCommunity extends SequentialCommandGroup {
    // Variables
    private SwerveSubsystem m_Drivetrain;

    public AutoGroup_LeaveCommunity(SwerveSubsystem drivetrain) {
        System.out.println("AutoGroup_LeaveCommunity");
        // Adding a drivetrain
        m_Drivetrain = drivetrain;
        // Adding Order of commands
        addCommands();
    }
}
