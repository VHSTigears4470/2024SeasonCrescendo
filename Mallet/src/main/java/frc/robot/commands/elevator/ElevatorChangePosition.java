// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Increments or decrements the elevator height by incAmt.
 */
public class ElevatorChangePosition extends Command {

  private final ElevatorSubsystem elevator;
  private final double incAmt;

  public ElevatorChangePosition(ElevatorSubsystem elevator, double incAmt) {
    this.elevator = elevator;
    this.incAmt = incAmt;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.changePosition(incAmt);
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