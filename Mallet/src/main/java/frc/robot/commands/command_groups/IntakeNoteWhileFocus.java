package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.commands.drivebase.DriveForwardWithFocus;
import frc.robot.commands.drivebase.CenterToFaceNote;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.IntakePositionExtend;
import frc.robot.commands.intake.IntakePusherRetract;
import frc.robot.commands.intake.IntakeSetIntakeVoltageEndWithBreakbeam;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteLimelight;
import frc.robot.subsystems.SwerveSubsystem;

public class IntakeNoteWhileFocus extends SequentialCommandGroup {
    public IntakeNoteWhileFocus(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, NoteLimelight limelight) {
        // Does nothing if a note is already in the intake
        if(!intakeSubsystem.getNoteBreakbeam()) {
            addCommands(
                // Sets robot to intake position
                new ParallelCommandGroup(
                    new IntakePusherRetract(intakeSubsystem),
                    new IntakePositionExtend(intakeSubsystem),
                    new ElevatorSetHeightState(elevatorSubsystem, ELEVATOR_STATE.DOWN),
                    new IntakeSetIntakeVoltageEndWithBreakbeam(intakeSubsystem)
                ),
                // Centers robot before going
                new CenterToFaceNote(swerveSubsystem, limelight),
                
                // Go forward with adjustments until robot is gets a note
                new ParallelRaceGroup(
                    new IntakeNote(intakeSubsystem),
                    new DriveForwardWithFocus(swerveSubsystem, limelight)
                )
            );
        }
    }
}