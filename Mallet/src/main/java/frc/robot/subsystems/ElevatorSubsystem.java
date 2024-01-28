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
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATES;

public class ElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax leadMotor;
    private final CANSparkMax followMotor;

    private final RelativeEncoder encoder;
    private ELEVATOR_STATES desiredState;
    private double marginOfError; 

    private final SparkPIDController pidController;

    private final DigitalInput bottomBreakBeam; // Assuming Limit Switch is used to set when elevator reaches bottom
    private double highestPos; // Highest encoder value the motor can go
    private double lowestPos; // Lowest encoder value the motor can go

    private ShuffleboardTab shuffleDebugTab;
    private GenericEntry entry_breakBeam;
    private GenericEntry entry_encoder;
    private GenericEntry entry_pid;
    // private GenericEntry 

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
    desiredState = ELEVATOR_STATES.DOWN;
    marginOfError = 0;
    highestPos = 0;
    lowestPos = 0;

    // Set up PID
    pidController.setP(ElevatorConstants.PID_KP);
    pidController.setI(ElevatorConstants.PID_KP);
    pidController.setD(ElevatorConstants.PID_KP);
    pidController.setIZone(ElevatorConstants.PID_KP);
    pidController.setOutputRange(ElevatorConstants.PID_KMIN_OUTPUT, ElevatorConstants.PID_KMAX_OUTPUT);

    // Smart Motion
    pidController.setSmartMotionMaxVelocity(ElevatorConstants.PID_MAX_VEL, ElevatorConstants.SMART_MOTION_SLOT);
    pidController.setSmartMotionMinOutputVelocity(ElevatorConstants.PID_MIN_VEL, ElevatorConstants.SMART_MOTION_SLOT);
    pidController.setSmartMotionMaxAccel(ElevatorConstants.PID_MAX_ACCEL, ElevatorConstants.SMART_MOTION_SLOT);
    pidController.setSmartMotionAllowedClosedLoopError(ElevatorConstants.PID_ALLOWED_ERR, ElevatorConstants.SMART_MOTION_SLOT);

    // Init Shuffleboard
    initalizeShuffleboard();
  }

  // Maintains the current state of the elevator and prevents it from falling/or raising 
  public void maintainState(){
    if(bottomBreakBeam.get())
        lowestPos = encoder.getPosition();
    if(desiredState == ELEVATOR_STATES.UP)
        pidController.setReference(highestPos, ControlType.kSmartMotion);
    else if(desiredState == ELEVATOR_STATES.DOWN){
        pidController.setReference(lowestPos, ControlType.kSmartMotion);
        if(encoder.getPosition() - lowestPos < marginOfError)
          leadMotor.setVoltage(0);
    }
  }

  // Stops motors in case of emergency
  public void emergencyStop(){
    leadMotor.stopMotor();
    followMotor.stopMotor();
  }

  // Sets the elevator to the current state
  public void setDesiredState(ELEVATOR_STATES desiredState){
    this.desiredState = desiredState;
  }

  // Gets the elevator's current state
  public ELEVATOR_STATES getDesiredState(){
    return desiredState;
  }

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
      // Make sure if you want to modify PID during run-time, enable in test mode
      entry_pid = shuffleDebugTab.getLayout("Elevator", BuiltInLayouts.kList)
        .add("PID Settings", pidController)
        .withWidget(BuiltInWidgets.kPIDController)
        .getEntry();
    }
  }

  private void updateShuffleboard(){
    if(ElevatorConstants.DEBUG){
      entry_breakBeam.setBoolean(bottomBreakBeam.get());
      entry_encoder.setDouble(encoder.getPosition());
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
