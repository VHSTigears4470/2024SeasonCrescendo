package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DifferentialConstants;

public class DifferentialSubsystem extends SubsystemBase {
    // Variables for drive motors
    private final Spark flMotor;
    private final Spark blMotor;
    private final Spark frMotor;
    private final Spark brMotor;
    private final DifferentialDrive diffDrive;

    public DifferentialSubsystem() {
        flMotor = new Spark(DifferentialConstants.FL_MOTOR_PWM);
        blMotor = new Spark(DifferentialConstants.BL_MOTOR_PWM);
        frMotor = new Spark(DifferentialConstants.FR_MOTOR_PWM);
        brMotor = new Spark(DifferentialConstants.BR_MOTOR_PWM);

        flMotor.addFollower(blMotor);
        flMotor.setInverted(true);
        frMotor.addFollower(brMotor);
        frMotor.setInverted(false);

        diffDrive = new DifferentialDrive(flMotor, frMotor);
    }

    public void arcadeDrive(double fwd, double rot) {
        diffDrive.arcadeDrive(fwd, rot);
    }

    public void stopMotors() {
        diffDrive.arcadeDrive(0, 0);
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