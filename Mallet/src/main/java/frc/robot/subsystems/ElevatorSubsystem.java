// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.util.ListDebugEntryString;
import frc.robot.util.ListDebugEntryDouble;
import frc.robot.util.ListDebugEntryBool;

public class ElevatorSubsystem extends SubsystemBase {

  /** Elevator left, Lead motor */
  private final CANSparkMax leftMotor;
  private final RelativeEncoder leftEncoder;

  /** Elevator right, Follower motor, */
  private final CANSparkMax rightMotor;
  private final RelativeEncoder rightEncoder;

  private final SparkPIDController pidController;

  /** false when blocked, true when open */
  private final DigitalInput bottomBreakBeam; // Assuming Limit Switch is used to set when elevator reaches bottom
  /** false when blocked, true when open */
  private final DigitalInput topBreakBeam; // Assuming Limit Switch is used to set when elevator reaches top

  /** bottom breakbeam toggle for only setting voltage to zero once */
  private boolean botBreakbeamTripped;
  /** top breakbeam toggle for only setting voltage to zero once */
  private boolean topBreakbeamTripped;

  private double highestPos; // Highest encoder value the motor can go
  private double lowestPos; // Lowest encoder value the motor can go
  private double desiredReferencePosition; // Desired position the motor should go to
  private ELEVATOR_STATE currState;
  // Shuffleboard
  private ShuffleboardTab shuffleDebugTab;
  private ShuffleboardTab shuffleDriverTab;

  private ListDebugEntryBool entry_botBeam;
  private ListDebugEntryBool entry_topBeam;
  private ListDebugEntryDouble entry_minLimit;
  private ListDebugEntryDouble entry_maxLimit;
  private ListDebugEntryDouble entry_desiredPosition;
  private ListDebugEntryDouble entry_leftEncoder;
  private ListDebugEntryDouble entry_rightEncoder;

  // PID Entries
  private ListDebugEntryDouble entry_pid_kp;
  private ListDebugEntryDouble entry_pid_ki;
  private ListDebugEntryDouble entry_pid_kd;
  private ListDebugEntryDouble entry_pid_kiz;
  private ListDebugEntryDouble entry_pid_kff;
  private ListDebugEntryDouble entry_smart_motion_allowed_err;
  // Driver Tab
  private ListDebugEntryString entry_elevatorState;

  /**
   * Initialize Elevator Subsystem
   */
  public ElevatorSubsystem() {
    // Init motors and positions
    leftMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    leftEncoder.setPosition(0);
    leftEncoder.setPositionConversionFactor(ElevatorConstants.CONVERSION_RATIO);

    rightMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    rightEncoder = rightMotor.getEncoder();
    rightEncoder.setPosition(0);
    rightEncoder.setPositionConversionFactor(ElevatorConstants.CONVERSION_RATIO);

    // Set inversions
    rightMotor.follow(leftMotor, ElevatorConstants.FOLLOWER_INVERTED);
    leftMotor.setInverted(ElevatorConstants.DIRECTION_INVERTED);

    botBreakbeamTripped = false;
    topBreakbeamTripped = false;
    lowestPos = ElevatorConstants.LOW_INIT_HEIGHT;
    highestPos = ElevatorConstants.HIGH_INIT_HEIGHT;
    currState = ELEVATOR_STATE.DOWN;

    // Init sensors
    bottomBreakBeam = new DigitalInput(ElevatorConstants.BOTTOM_BREAKBEAM_DIO);
    topBreakBeam = new DigitalInput(ElevatorConstants.TOP_BREAKBEAM_DIO);

    // Init PID
    pidController = leftMotor.getPIDController();
    pidController.setP(ElevatorConstants.PID_KP);
    pidController.setI(ElevatorConstants.PID_KI);
    pidController.setD(ElevatorConstants.PID_KD);
    pidController.setIZone(ElevatorConstants.PID_KIZ);
    pidController.setOutputRange(ElevatorConstants.PID_KMIN_OUTPUT, ElevatorConstants.PID_KMAX_OUTPUT);

    // Init Smart Motion
    pidController.setSmartMotionMaxVelocity(ElevatorConstants.SM_MAX_RPM_VEL, ElevatorConstants.SM_ID);
    pidController.setSmartMotionMinOutputVelocity(ElevatorConstants.SM_MIN_RPM_VEL,
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
    } else if (tempPos > highestPos) {
      desiredReferencePosition = highestPos;
    } else {
      desiredReferencePosition = tempPos;
    }
    pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
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
    leftMotor.stopMotor();
    rightMotor.stopMotor();
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
    } else if (desiredState == ELEVATOR_STATE.CLIMB) {
      currState = desiredState;
      desiredReferencePosition = ElevatorConstants.CLIMB_HEIGHT;
    }
    pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
  }

  /** Zeros encoders */
  public void zeroEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void handleBreakbeams() {
    if (botBreakbeamTripped()) {
      if (!botBreakbeamTripped) {
        zeroEncoders();
        desiredReferencePosition = 0;
        botBreakbeamTripped = true;
        leftMotor.setVoltage(0); // Stop motor
        pidController.setReference(lowestPos, ControlType.kSmartMotion);
      }
      pidController.setOutputRange(0, ElevatorConstants.PID_KMAX_OUTPUT); // Only allowed to go up
    } else if (!botBreakbeamTripped() && botBreakbeamTripped) {
      botBreakbeamTripped = false;
      pidController.setOutputRange(ElevatorConstants.PID_KMIN_OUTPUT, ElevatorConstants.PID_KMAX_OUTPUT);
    }
    if (topBreakbeamTripped()) {
      if (!topBreakbeamTripped) {
        highestPos = leftEncoder.getPosition();
        desiredReferencePosition = highestPos;
        topBreakbeamTripped = true;
        leftMotor.setVoltage(0); // Stop motor
        pidController.setReference(highestPos, ControlType.kSmartMotion);
      }
      pidController.setOutputRange(ElevatorConstants.PID_KMIN_OUTPUT, 0); // Only allowed to go down
    } else if (!topBreakbeamTripped() && topBreakbeamTripped) {
      topBreakbeamTripped = false;
      pidController.setOutputRange(ElevatorConstants.PID_KMIN_OUTPUT, ElevatorConstants.PID_KMAX_OUTPUT);
    }
  }

  /*** Returns the desired position of the motor */
  public double getDesiredReferencePosition() {
    return desiredReferencePosition;
  }

  /*** Returns the desired state of the motor */
  public ELEVATOR_STATE getDesiredState() {
    return currState;
  }

  /***
   * Returns true if the elevator is within margin of error of the desired
   * position
   */
  public boolean isAtPos() {
    return Math.abs(leftEncoder.getPosition() - desiredReferencePosition) < ElevatorConstants.POSITION_TOLERANCE;
  }

  public boolean botBreakbeamTripped() {
    return !bottomBreakBeam.get(); // inverts because false when blocked, true when open
  }

  public boolean topBreakbeamTripped() {
    return !topBreakBeam.get(); // inverts because false when blocked, true when open
  }

  // Init Shuffleboard
  private void initalizeShuffleboard() {
    shuffleDriverTab = Shuffleboard.getTab("Driver's Tab");

    entry_elevatorState = new ListDebugEntryString(shuffleDriverTab, getSubsystem(), "Elevator Height State", "Down",
        BuiltInWidgets.kTextView);
    if (ElevatorConstants.DEBUG) {
      shuffleDebugTab = Shuffleboard.getTab("Debug Tab");
      entry_botBeam = new ListDebugEntryBool(shuffleDebugTab, "Elevator", "Bottom Breakbeam Tripped", false,
          BuiltInWidgets.kBooleanBox);
      entry_topBeam = new ListDebugEntryBool(shuffleDebugTab, "Elevator", "Top Breakbeam Tripped", false,
          BuiltInWidgets.kBooleanBox);
      entry_minLimit = new ListDebugEntryDouble(shuffleDebugTab, "Elevator", "Min Limit", lowestPos,
          BuiltInWidgets.kTextView);
      entry_maxLimit = new ListDebugEntryDouble(shuffleDebugTab, "Elevator", "Max Limit", highestPos,
          BuiltInWidgets.kTextView);
      entry_desiredPosition = new ListDebugEntryDouble(shuffleDebugTab, "Elevator", "Desired Position",
          desiredReferencePosition,
          BuiltInWidgets.kTextView);
      entry_leftEncoder = new ListDebugEntryDouble(shuffleDebugTab, "Elevator", "Left Encoder Value", 0.0,
          BuiltInWidgets.kEncoder);
      entry_rightEncoder = new ListDebugEntryDouble(shuffleDebugTab, "Elevator", "Right Encoder Value", 0.0,
          BuiltInWidgets.kEncoder);

      // PID Init CURRENTLY DISABLED
      entry_pid_kp = new ListDebugEntryDouble(shuffleDebugTab, "Elevator PID", "P Gain", ElevatorConstants.PID_KP,
          BuiltInWidgets.kTextView, false);
      entry_pid_ki = new ListDebugEntryDouble(shuffleDebugTab, "Elevator PID", "I Gain", ElevatorConstants.PID_KI,
          BuiltInWidgets.kTextView, false);
      entry_pid_kd = new ListDebugEntryDouble(shuffleDebugTab, "Elevator PID", "D Gain", ElevatorConstants.PID_KD,
          BuiltInWidgets.kTextView, false);
      entry_pid_kiz = new ListDebugEntryDouble(shuffleDebugTab, "Elevator PID", "Iz Gain", ElevatorConstants.PID_KIZ,
          BuiltInWidgets.kTextView, false);
      entry_pid_kff = new ListDebugEntryDouble(shuffleDebugTab, "Elevator PID", "FF Gain", ElevatorConstants.PID_KFF,
          BuiltInWidgets.kTextView, false);

      // Smart Motion
      entry_smart_motion_allowed_err = new ListDebugEntryDouble(shuffleDebugTab, "Elevator PID", "Allowed Error",
          ElevatorConstants.SM_ALLOWED_ERR, BuiltInWidgets.kTextView, false);
    }
  }

  // Updates Shuffleboard
  private void updateShuffleboard() {
    entry_elevatorState.set(currState == ELEVATOR_STATE.DOWN ? "DOWN" : "UP");
    if (ElevatorConstants.DEBUG) {
      entry_botBeam.set(botBreakbeamTripped());
      entry_topBeam.set(topBreakbeamTripped());
      entry_minLimit.set(lowestPos);
      entry_maxLimit.set(highestPos);
      entry_desiredPosition.set(desiredReferencePosition);
      entry_leftEncoder.set(leftEncoder.getPosition());
      entry_rightEncoder.set(rightEncoder.getPosition());

      int slotId = ElevatorConstants.SM_ID;
      pidController.setP((double) entry_pid_kp.get(pidController.getP(slotId)), slotId);
      pidController.setI((double) entry_pid_ki.get(pidController.getI(slotId)), slotId);
      pidController.setD((double) entry_pid_kd.get(pidController.getD(slotId)), slotId);
      pidController.setIZone((double) entry_pid_kiz.get(pidController.getIZone(slotId)), slotId);
      pidController.setFF((double) entry_pid_kff.get(pidController.getFF(slotId)), slotId);

      pidController.setSmartMotionAllowedClosedLoopError(
          (double) entry_smart_motion_allowed_err.get(pidController.getSmartMotionAllowedClosedLoopError(slotId)),
          slotId);
    }
  }

  @Override
  public void periodic() {
    handleBreakbeams();
    updateShuffleboard();
    disabledPeriodic();
  }

  public void disabledPeriodic() {
    if (DriverStation.isDisabled()) {
      desiredReferencePosition = leftEncoder.getPosition();
      pidController.setReference(desiredReferencePosition, ControlType.kSmartMotion);
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  public void leadSetVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
  }
}
