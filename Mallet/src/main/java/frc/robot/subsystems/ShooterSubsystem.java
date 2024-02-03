//DO NOT MODIFY THIS FILE!
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  // Variables for motors
  private final CANSparkMax m_topMotor;
  private final CANSparkMax m_bottomMotor;
  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  // Shuffleboard
  private ShuffleboardTab shuffleDebugTab;
  private GenericEntry entry_topMotorIdealVoltage; // ideal is inputted voltage
  private GenericEntry entry_bottomMotorIdealVoltage;
  private GenericEntry entry_topMotorActualVoltage; // actual is voltage actually being used
  private GenericEntry entry_bottomMotorActualVoltage;

  public ShooterSubsystem() {
    // Top motor & encoder initialization
    m_topMotor = new CANSparkMax(ShooterConstants.TOP_MOTOR_CAN, MotorType.kBrushless);
    topEncoder = m_topMotor.getEncoder();
    m_topMotor.setIdleMode(IdleMode.kBrake);
    m_topMotor.setInverted(false);
    topEncoder.setPosition(0);
    // Bottom motor & encoder initialization
    m_bottomMotor = new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_CAN, MotorType.kBrushless);
    bottomEncoder = m_bottomMotor.getEncoder();
    m_bottomMotor.setIdleMode(IdleMode.kBrake);
    m_bottomMotor.setInverted(false);
    bottomEncoder.setPosition(0);
  }

  public void stop() {
    m_topMotor.setVoltage(0);
    m_bottomMotor.setVoltage(0);
  }

  public void forward(double voltage) {
    m_topMotor.setVoltage(voltage);
    m_bottomMotor.setVoltage(voltage);
  }

  public void backwards(double voltage) { // backwards(speed) = forwards(-speed) so probably will condense to 1 method
    m_topMotor.setVoltage(-voltage);
    m_bottomMotor.setVoltage(-voltage);
  }

  private void initializeShuffleboard() {
    if (IntakeConstants.DEBUG) {
      shuffleDebugTab = Shuffleboard.getTab("Debug Tab");
      entry_topMotorIdealVoltage = shuffleDebugTab.getLayout("Shooter", BuiltInLayouts.kList)
          .add("Top Motor Ideal Voltage", 0)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      entry_bottomMotorIdealVoltage = shuffleDebugTab.getLayout("Shooter", BuiltInLayouts.kList)
          .add("Bottom Motor Ideal Voltage", 0)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      entry_topMotorActualVoltage = shuffleDebugTab.getLayout("Shooter", BuiltInLayouts.kList)
          .add("Top Motor Actual Voltage", 0)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      entry_bottomMotorActualVoltage = shuffleDebugTab.getLayout("Shooter", BuiltInLayouts.kList)
          .add("Bottom Motor Actual Voltage", 0)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
    }
  }

  private void updateShuffleboard() {
    if (IntakeConstants.DEBUG) {
      entry_topMotorIdealVoltage.setDouble(m_topMotor.getBusVoltage());
      entry_bottomMotorIdealVoltage.setDouble(m_bottomMotor.getBusVoltage());
      entry_topMotorActualVoltage.setDouble(m_topMotor.getBusVoltage() * m_topMotor.getAppliedOutput());
      entry_bottomMotorActualVoltage.setDouble(m_bottomMotor.getBusVoltage() * m_bottomMotor.getAppliedOutput());
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