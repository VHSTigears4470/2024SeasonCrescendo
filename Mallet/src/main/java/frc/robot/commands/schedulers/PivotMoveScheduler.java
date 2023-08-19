package frc.robot.commands.schedulers;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.pivot.PivotMove;
import frc.robot.subsystems.PivotSub;


public class PivotMoveScheduler extends CommandBase{
    // Required Subsystems
    private PivotSub m_pivot;

    // Creation Function of the Class
    public PivotMoveScheduler(PivotSub pivot){
        m_pivot = pivot;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        new PivotMove(m_pivot).schedule();
    }

    // Schedules pivot default movement
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    // Nothing is called here as it is covered already in the subsystem to stop the motor.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
