// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;

public class ElevatorSubsystem extends SubsystemBase {

  private final CANSparkMax leadMotor;
  private final CANSparkMax followMotor;

  private final RelativeEncoder encoder;

  private final SparkPIDController pidController;

  private final DigitalInput bottomBreakBeam; // Assuming Limit Switch is used to set when elevator reaches bottom
  private double highestPos; // Highest encoder value the motor can go
  private double lowestPos; // Lowest encoder value the motor can go
  private double desiredReferencePosition; // Desired position the motor should go to
  private ELEVATOR_STATE currState;
  // Shuffleboard
  private ShuffleboardTab shuffleDebugTab;
  private ShuffleboardTab shuffleDriverTab;
  private GenericEntry entry_breakBeam;
  private GenericEntry entry_encoder;
  private GenericEntry entry_desiredPosition;
  // PID Entries
  private GenericEntry entry_pid_kp;
  private GenericEntry entry_pid_ki;
  private GenericEntry entry_pid_kd;
  private GenericEntry entry_pid_kiz;
  private GenericEntry entry_pid_kff;
  private GenericEntry entry_pid_max_output;
  private GenericEntry entry_smart_motion_allowed_err;
  private GenericEntry entry_smart_motion_pos;
  // Driver Tab
  private GenericEntry entry_elevatorState;

  /**
   * Initialize Elevator Subsystem
   */
  public ElevatorSubsystem() {
    // Init motors and positions
    leadMotor = new CANSparkMax(ElevatorConstants.LEAD_MOTOR_ID, MotorType.kBrushless);
    followMotor = new CANSparkMax(ElevatorConstants.FOLLOW_MOTOR_ID, MotorType.kBrushless);
    followMotor.follow(leadMotor, true);
    pidController = leadMotor.getPIDController();

    encoder = leadMotor.getEncoder();
    encoder.setPosition(0);
    encoder.setPositionConversionFactor(ElevatorConstants.CONVERSION_RATIO);

    lowestPos = ElevatorConstants.LOW_INIT_HEIGHT;
    highestPos = ElevatorConstants.HIGH_INIT_HEIGHT;
    currState = ELEVATOR_STATE.DOWN;

    // Init sensors
    bottomBreakBeam = new DigitalInput(ElevatorConstants.BOTTOM_BREAKBEAM_CHANNEL_ID);

    // Init PID
    pidController.setP(ElevatorConstants.PID_KP);
    pidController.setI(ElevatorConstants.PID_KI);
    pidController.setD(ElevatorConstants.PID_KD);
    pidController.setIZone(ElevatorConstants.PID_KIZ);
    pidController.setOutputRange(ElevatorConstants.PID_KMIN_OUTPUT, ElevatorConstants.PID_KMAX_OUTPUT);

    // Init Smart Motion
    pidController.setSmartMotionMaxVelocity(ElevatorConstants.SM_MAX_RPM_VEL, ElevatorConstants.SM_ID);
    pidController.setSmartMotionMinOutputVelocity(ElevatorConstants.SM_MIN_RPM_OUTPUT_VEL,
        ElevatorConstants.SM_ID);
    pidController.setSmartMotionMaxAccel(ElevatorConstants.SM_MAX_RPM_ACC, ElevatorConstants.SM_ID);
    pidController.setSmartMotionAllowedClosedLoopError(ElevatorConstants.SM_ALLOWED_ERR,
        ElevatorConstants.SM_ID);

    // Init Shuffleboard
    initalizeShuffleboard();
  }

  /*** Change position of elevator in inches by amt (+ up, - down) */
  public void changePosition(double amt) {
    double tempPos = desiredReferencePosition + amt;
    if (tempPos < lowestPos) {
      desiredReferencePosition = lowestPos;
    } else {
      desiredReferencePosition = tempPos;
    }

    pidController.setReference(tempPos, ControlType.kSmartMotion);
  }

  /***
   * Change position of elevator in inches by amt (+ up, - down), ignores soft
   * limit to home to breakbeam
   */
  public void changePositionIgnoreSoftLimit(double amt) {
    desiredReferencePosition += amt;
    pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
  }

  /*** Stops motors in case of emergency */
  public void emergencyStop() {
    leadMotor.stopMotor();
    followMotor.stopMotor();
  }

  /*** Sets elevator to desired height */
  public void setPosition(double height) {
    double tempPos = desiredReferencePosition;
    if (tempPos < lowestPos) {
      tempPos = lowestPos;
    }
  }

  /*** Sets the elevator to the desired state */
  public void setHeightState(ELEVATOR_STATE desiredState) {
    if (desiredState == ELEVATOR_STATE.UP) {
      currState = desiredState;
      desiredReferencePosition = highestPos;
    } else if (desiredState == ELEVATOR_STATE.DOWN) {
      currState = desiredState;
      desiredReferencePosition = lowestPos;
    }
    pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
  }

  /*** Returns the desired position of the motor */
  public double getDesiredReferencePosition() {
    return desiredReferencePosition;
  }

  /***
   * Returns true if the elevator is within margin of error of the desired
   * position
   */
  public boolean isAtPos() {
    return Math.abs(encoder.getPosition() - desiredReferencePosition) < ElevatorConstants.POSITION_TOLERANCE;
  }

  public boolean getBottomBreakbeam() {
    return bottomBreakBeam.get();
  }

  // Init Shuffleboard
  private void initalizeShuffleboard() {
    shuffleDriverTab = Shuffleboard.getTab("Driver's Tab");
    entry_elevatorState = shuffleDriverTab.getLayout("Elevator", BuiltInLayouts.kList)
        .add("Elevator Height State", "Down").withWidget(BuiltInWidgets.kTextView).getEntry();
    if (ElevatorConstants.DEBUG) {
      shuffleDebugTab = Shuffleboard.getTab("Debug Tab");
      entry_breakBeam = shuffleDebugTab.getLayout("Elevator", BuiltInLayouts.kList)
          .add("IR Break Beam Sensor", false)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .getEntry();
      entry_encoder = shuffleDebugTab.getLayout("Elevator", BuiltInLayouts.kList)
          .add("Encoder Value", 0)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();

      // PID Init
      entry_pid_kp = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
          .add("P Gain", ElevatorConstants.PID_KP)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      entry_pid_ki = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
          .add("I Gain", ElevatorConstants.PID_KI)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      entry_pid_kd = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
          .add("D Gain", ElevatorConstants.PID_KD)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      entry_pid_kiz = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
          .add("Iz Gain", ElevatorConstants.PID_KIZ)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      entry_pid_kff = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
          .add("FF Gain", ElevatorConstants.PID_KFF)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      entry_pid_max_output = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
          .add("Max Output", ElevatorConstants.PID_KMAX_OUTPUT)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();

      // Smart Motion
      entry_smart_motion_allowed_err = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
          .add("Allowed Error", ElevatorConstants.SM_ALLOWED_ERR)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
      entry_smart_motion_pos = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
          .add("Position", encoder.getPosition())
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
    }
  }

  // Updates Shuffleboard
  private void updateShuffleboard() {
    entry_elevatorState.setString(currState == ELEVATOR_STATE.DOWN ? "DOWN" : "UP");
    if (ElevatorConstants.DEBUG) {
      entry_breakBeam.setBoolean(bottomBreakBeam.get());
      entry_encoder.setDouble(encoder.getPosition());
      entry_smart_motion_pos.setDouble(desiredReferencePosition);

      int slotId = ElevatorConstants.SM_ID;
      pidController.setP(entry_pid_kp.getDouble(pidController.getP(slotId)), slotId);
      pidController.setI(entry_pid_ki.getDouble(pidController.getI(slotId)), slotId);
      pidController.setD(entry_pid_kd.getDouble(pidController.getD(slotId)), slotId);
      pidController.setIZone(entry_pid_kiz.getDouble(pidController.getIZone(slotId)), slotId);
      pidController.setFF(entry_pid_kff.getDouble(pidController.getFF(slotId)), slotId);
      pidController.setOutputRange(ElevatorConstants.PID_KMIN_OUTPUT,
          entry_pid_max_output.getDouble(pidController.getOutputMax(slotId)), slotId);

      pidController.setSmartMotionAllowedClosedLoopError(
          entry_smart_motion_allowed_err.getDouble(pidController.getSmartMotionAllowedClosedLoopError(slotId)), slotId);
    }
  }

  @Override
  public void periodic() {
    if (getBottomBreakbeam()) {
      lowestPos = encoder.getPosition() + ElevatorConstants.HIGH_LOW_OFFSET;
    }
    updateShuffleboard();
  }

  @Override
  public void simulationPeriodic() {
  }
}
