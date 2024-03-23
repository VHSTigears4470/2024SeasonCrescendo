package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivebase.DriveForwardWithFocus;
import frc.robot.commands.drivebase.FaceNoteInstant;
import frc.robot.commands.intake.IntakeSetIntakeVoltageEndWithBreakbeam;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteLimelight;
import frc.robot.subsystems.SwerveSubsystem;

public class IntakeNoteWhileFocus extends SequentialCommandGroup {
    public IntakeNoteWhileFocus(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem,
            ElevatorSubsystem elevatorSubsystem, NoteLimelight limelight) {
        // Does nothing if a note is already in the intake
        if (intakeSubsystem.noteBreambeamTripped()) {
            addCommands(
                    // Centers while going to intake position, wait command used to ensure enough
                    // time to set up intake position
                    new ParallelCommandGroup(
                            // Sets robot to intake position
                            new IntakePosition(intakeSubsystem, elevatorSubsystem),
                            // Centers robot before going
                            new FaceNoteInstant(swerveSubsystem, limelight),
                            new WaitCommand(0.5) // TODO - Link this to a constant in Constants.java
                    ),
                    // Go forward with adjustments until robot is gets a note
                    new ParallelRaceGroup(
                            new IntakeSetIntakeVoltageEndWithBreakbeam(intakeSubsystem),
                            new DriveForwardWithFocus(swerveSubsystem, limelight),
                            new WaitCommand(3) // Deadline
                    ));
        }
    }
}