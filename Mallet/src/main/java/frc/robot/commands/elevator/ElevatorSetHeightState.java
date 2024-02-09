// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Sets height to desired up or down state.
 */
public class ElevatorSetHeightState extends Command {

  private final ElevatorSubsystem elevator;
  private final ELEVATOR_STATE desiredState;

  public ElevatorSetHeightState(ElevatorSubsystem elevator, ELEVATOR_STATE desiredState) {
    this.elevator = elevator;
    this.desiredState = desiredState;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setHeightState(desiredState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}