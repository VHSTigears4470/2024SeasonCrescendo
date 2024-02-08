// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.INTAKE_SUBSYSTEM;
import frc.robot.subsystems.IntakeSubsystem;
/**
 * Increments or decrements the elevator height by incAmt.
 */
public class IntakeRetract extends Command {
  
  private final IntakeSubsystem intake;
  private final INTAKE_STATE intakeState;   

  public IntakeRetract(IntakeSubsystem intake, INTAKE_STATE state){
    this.intake = intake;
    this.intakeState = state;
    addRequirements(intake);

}