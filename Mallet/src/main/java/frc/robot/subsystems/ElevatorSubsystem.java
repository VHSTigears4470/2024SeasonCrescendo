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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATES;

public class ElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax leadMotor;
    private final CANSparkMax followMotor;

    private final RelativeEncoder encoder;
    private double desiredPos;
    private double marginOfError; 

    private final SparkPIDController pidController;

    private final DigitalInput bottomBreakBeam; // Assuming Limit Switch is used to set when elevator reaches bottom
    private double highestPos; // Highest encoder value the motor can go
    private double lowestPos; // Lowest encoder value the motor can go

    private ShuffleboardTab shuffleDebugTab;
    private GenericEntry entry_breakBeam;
    private GenericEntry entry_encoder;
    // PID Entries
    private GenericEntry entry_pid_kp;
    private GenericEntry entry_pid_ki;
    private GenericEntry entry_pid_kd;
    private GenericEntry entry_pid_kiz;
    private GenericEntry entry_pid_kff;
    private GenericEntry entry_pid_max_output;
    private GenericEntry entry_pid_min_output;
    // Smart Motion Entries 
    private GenericEntry entry_smart_motion_max_vel;
    private GenericEntry entry_smart_motion_min_vel;
    private GenericEntry entry_smart_motion_max_acc;
    private GenericEntry entry_smart_motion_allowed_err;
    private GenericEntry entry_smart_motion_pos;
    private GenericEntry entry_smart_motion_slot;

  /**
   * Initialize Elevator Subsystem
   */
  public ElevatorSubsystem() {
    // Init variables
    leadMotor = new CANSparkMax(ElevatorConstants.LEAD_MOTOR_ID, MotorType.kBrushless);
    followMotor = new CANSparkMax(ElevatorConstants.FOLLOW_MOTOR_ID, MotorType.kBrushless);
    encoder = leadMotor.getEncoder();
    pidController = leadMotor.getPIDController();
    bottomBreakBeam = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID);
    // Set up motors and positions
    followMotor.follow(leadMotor, true);
    encoder.setPosition(0);
    desiredPos = 0;
    marginOfError = 0; // TODO: Update
    highestPos = 0; // TODO: Update
    lowestPos = 0; // TODO: Update

    // Set up PID
    pidController.setP(ElevatorConstants.PID_KP);
    pidController.setI(ElevatorConstants.PID_KP);
    pidController.setD(ElevatorConstants.PID_KP);
    pidController.setIZone(ElevatorConstants.PID_KP);
    pidController.setOutputRange(ElevatorConstants.PID_KMIN_OUTPUT, ElevatorConstants.PID_KMAX_OUTPUT);

    // Smart Motion
    pidController.setSmartMotionMaxVelocity(ElevatorConstants.SMART_MOTION_MAX_VEL, ElevatorConstants.SMART_MOTION_ID);
    pidController.setSmartMotionMinOutputVelocity(ElevatorConstants.SMART_MOTION_MIN_VEL, ElevatorConstants.SMART_MOTION_ID);
    pidController.setSmartMotionMaxAccel(ElevatorConstants.SMART_MOTION_MAX_ACC, ElevatorConstants.SMART_MOTION_ID);
    pidController.setSmartMotionAllowedClosedLoopError(ElevatorConstants.SMART_MOTION_ALLOWED_ERR, ElevatorConstants.SMART_MOTION_ID);

    // Init Shuffleboard
    initalizeShuffleboard();
  }

  // Maintains the current state of the elevator and prevents it from falling/or raising 
  public void maintainState(){
    // When the bottom break beam is triggered, the elevator has reached its bottom
    if(bottomBreakBeam.get()){
        lowestPos = encoder.getPosition();
        desiredPos = lowestPos;
    }
    pidController.setReference(highestPos, ControlType.kSmartMotion);
    // Turns off the motors when the elevator has reached the bottom
    if(Math.abs(encoder.getPosition() - lowestPos) < marginOfError)
        leadMotor.setVoltage(0);
  }

  public Command maintainStateCommand(){
    return run(() -> {
      maintainState();
    });
  }

  // Increments position of the elevator and positive for inc amount assumes upwards
  public void incrementPos(double incAmt){
    // Check to make sure the incAmt won't cause the elevator to go out of bounds
    if(encoder.getPosition() + incAmt > highestPos || encoder.getPosition() + incAmt < lowestPos)
      incAmt = 0;
    desiredPos += incAmt;
  }

  public Command incrementPosCommand(double incAmt){
    return runOnce(() -> {
      incrementPos(incAmt);
    });
  }

  // Stops motors in case of emergency
  public void emergencyStop(){
    leadMotor.stopMotor();
    followMotor.stopMotor();
  }

  // Sets the elevator to the current state
  public void setDesiredState(ELEVATOR_STATES desiredState){
    if(desiredState == ELEVATOR_STATES.UP)
      desiredPos = highestPos;
    else if(desiredState == ELEVATOR_STATES.DOWN)
      desiredPos = lowestPos;
  }

  public Command setDesiredStateCommand(ELEVATOR_STATES desirdState){
    return runOnce(() -> {
      setDesiredState(desirdState);
    });
  }

  // Returns the desired position of the motor
  public double getDesiredPos(){
    return desiredPos;
  }

  // Returns if the elevator is a the desired position
  public boolean isAtPos(){
    return Math.abs(encoder.getPosition() - desiredPos) < marginOfError;
  }

  // Init Shuffleboard
  private void initalizeShuffleboard(){
    if(ElevatorConstants.DEBUG){
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
      entry_pid_min_output = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
        .add("Min Output", ElevatorConstants.PID_KMIN_OUTPUT)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

      // Smart Motion
      entry_smart_motion_max_vel = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
        .add("Max Vel", ElevatorConstants.SMART_MOTION_MAX_VEL)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
      entry_smart_motion_max_vel = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
        .add("Min Vel", ElevatorConstants.SMART_MOTION_MIN_VEL)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
      entry_smart_motion_max_acc = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
        .add("Min Accel", ElevatorConstants.SMART_MOTION_MAX_ACC)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
      entry_smart_motion_allowed_err = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
        .add("Allowed Error", ElevatorConstants.SMART_MOTION_ALLOWED_ERR)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
     entry_smart_motion_pos = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
        .add("Position", encoder.getPosition())
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
      entry_smart_motion_slot = shuffleDebugTab.getLayout("Elevator PID", BuiltInLayouts.kList)
        .add("Slot ID", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    }
  }

  // Updates Shuffleboard
  private void updateShuffleboard(){
    if(ElevatorConstants.DEBUG){
      entry_breakBeam.setBoolean(bottomBreakBeam.get());
      entry_encoder.setDouble(encoder.getPosition());
      entry_smart_motion_pos.setDouble(desiredPos);

      int slotId = (int) entry_smart_motion_slot.getInteger(0);
      pidController.setP(entry_pid_kp.getDouble(pidController.getP(slotId)), slotId);
      pidController.setI(entry_pid_ki.getDouble(pidController.getI(slotId)), slotId);
      pidController.setD(entry_pid_kd.getDouble(pidController.getD(slotId)), slotId);
      pidController.setIZone(entry_pid_kiz.getDouble(pidController.getIZone(slotId)), slotId);
      pidController.setFF(entry_pid_kff.getDouble(pidController.getFF(slotId)), slotId);
      pidController.setOutputRange(entry_pid_min_output.getDouble(pidController.getOutputMin(slotId)), entry_pid_max_output.getDouble(pidController.getOutputMax(slotId)), slotId);

      pidController.setSmartMotionMaxVelocity(entry_smart_motion_max_vel.getDouble(pidController.getSmartMotionMaxVelocity(slotId)), slotId);
      pidController.setSmartMotionMinOutputVelocity(entry_smart_motion_min_vel.getDouble(pidController.getSmartMotionMinOutputVelocity(slotId)), slotId);
      pidController.setSmartMotionMaxAccel(entry_smart_motion_max_acc.getDouble(pidController.getSmartMotionMaxAccel(slotId)), slotId);
      pidController.setSmartMotionAllowedClosedLoopError(entry_smart_motion_allowed_err.getDouble(pidController.getSmartMotionAllowedClosedLoopError(slotId)), slotId);
    }
  }

  @Override
  public void periodic() {
    updateShuffleboard();
  }

  @Override
  public void simulationPeriodic() {
  }
}
