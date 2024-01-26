package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    // Variables for intake pneumatics deployment and retractaction
    private final Compressor compressor;
    private final DoubleSolenoid doubleSolenoid;

    public IntakeSubsystem() {
        // Pneumatics initialization
        compressor = new Compressor(IntakeConstants.PCM_MODULE_ID, IntakeConstants.MODULE_TYPE);
        doubleSolenoid = new DoubleSolenoid(IntakeConstants.PCM_MODULE_ID, IntakeConstants.MODULE_TYPE,
                IntakeConstants.REVERSE_CHANNEL_ID,
                IntakeConstants.FORWARD_CHANNEL_ID);
        doubleSolenoid.set(IntakeConstants.DEFAULT_POSITION);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
