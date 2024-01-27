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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATES;

public class ElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax leadMotor = new CANSparkMax(ElevatorConstants.LEAD_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax followMotor = new CANSparkMax(ElevatorConstants.FOLLOW_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder encoder = leadMotor.getEncoder();
    private ELEVATOR_STATES desiredState;

    private final SparkPIDController pidController = leadMotor.getPIDController();

    private final DigitalInput bottomSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID); // Assuming Limit Switch is used to set when elevator reaches bottom
    private double lowestPos = 0; // Highest encoder value the motor can go
    private double highestPos = 0; // Lowest encoder value the motor can go

  /**
   * Initialize Elevator Subsystem
   */
  public ElevatorSubsystem() {
    // Set up motors
    followMotor.follow(leadMotor, true);

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

    desiredState = ELEVATOR_STATES.DOWN;
  }

  // Maintains the current state of the elevator and prevents it from falling
  public void maintainState(){
    if(!bottomSwitch.get())
        lowestPos = encoder.getPosition();
    if(desiredState == ELEVATOR_STATES.UP)
        pidController.setReference(highestPos, ControlType.kSmartMotion);
    else if(desiredState == ELEVATOR_STATES.DOWN)
        pidController.setReference(lowestPos, ControlType.kSmartMotion);
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

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
