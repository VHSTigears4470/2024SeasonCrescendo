//DO NOT MODIFY THIS FILE!
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  // Variables for motors
  private final Spark flywheelMotor;
  private final Spark feederMotor;

  // Shuffleboard
  private ShuffleboardTab shuffleDebugTab;

  public ShooterSubsystem() {
    // Flyhweel initialization
    flywheelMotor = new Spark(ShooterConstants.FLYWHEEL_ID);
    flywheelMotor.setInverted(false);
    // Feeder initialization
    feederMotor = new Spark(ShooterConstants.FEEDER_ID);
    feederMotor.setInverted(false);
    // Init shuffleboard
    initializeShuffleboard();
  }

  public void stop() {
    flywheelMotor.setVoltage(0);
    feederMotor.setVoltage(0);
  }

  public void forward(double voltage) {
    flywheelMotor.setVoltage(voltage);
    feederMotor.setVoltage(voltage);
  }

  public void backwards(double voltage) { // backwards(speed) = forwards(-speed) so probably will condense to 1 method
    flywheelMotor.setVoltage(-voltage);
    feederMotor.setVoltage(-voltage);
  }

  public void moveFlywheel(double voltage) {
    flywheelMotor.set(voltage);
  }

  public void moveFeeder(double voltage) {
    feederMotor.set(voltage);
  }

  private void initializeShuffleboard() {
    if (IntakeConstants.DEBUG) {
      shuffleDebugTab = Shuffleboard.getTab("Debug Tab");
    }
  }

  private void updateShuffleboard() {
    if (ShooterConstants.DEBUG) {
    }
  }
  // LoBOTTOMy ( sorry i came up with this pun cause of my neighbor, they really
  // love their loBOTTOMies ).

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}