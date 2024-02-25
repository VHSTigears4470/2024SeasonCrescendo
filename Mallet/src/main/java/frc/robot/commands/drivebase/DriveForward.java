package frc.robot.commands.drivebase;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** Default drive command for differential drive. */
public class DriveForward extends Command {
    private final SwerveSubsystem sub;
    private final double velocity;
    private ChassisSpeeds chassisSpeed;

    /**
     * Goes forward in the direction robot is facing.
     * Note this command will never finish
     *
     * @param subsystem The subsystem used by this command.
     * @param velocity The speed of the swerve in meters per second
     */
    public DriveForward(SwerveSubsystem sub, double velocity) {
        this.sub = sub;
        this.velocity = velocity;
        addRequirements(sub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Sets the "forward" motion based on rotation

        // Based on sim, cos is forward and sin is sideways
        // Cos and Sin may or may not need to be swapped, these values are purely from sim
        chassisSpeed = new ChassisSpeeds(sub.getHeading().getCos() * velocity, sub.getHeading().getSin() * velocity, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Also based on sim, if using a drive function that needs a heading,
        // rotation from swerve may be too presice and cause deviate the longer this command runs
        // as a result use (double)(Math.round(sub.getHeading().getRadians() * 100)) / 100 
        // for almost no deviation
        sub.drive(chassisSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stops the swerve
        sub.drive(new ChassisSpeeds(0, 0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}