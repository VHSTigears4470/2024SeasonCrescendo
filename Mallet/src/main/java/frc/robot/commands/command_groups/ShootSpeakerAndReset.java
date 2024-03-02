package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakePusherExtend;
import frc.robot.commands.intake.IntakePositionRetract;
import frc.robot.commands.intake.IntakeSetSpeakerVoltage;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootSpeakerAndReset extends SequentialCommandGroup {
    public ShootSpeakerAndReset(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        // TODO: precondition to do nothing if a note is not already in the intake
        addCommands(
                // TODO: Light on the robot glows when in correct range/radius around the
                // speaker and pointing in correct direction
                new IntakePositionRetract(intakeSubsystem),
                new WaitCommand(0.7),
                new IntakeSetSpeakerVoltage(intakeSubsystem),
                new WaitCommand(0.5),
                new IntakePusherExtend(intakeSubsystem),
                new WaitCommand(0.5),
                new DefaultPosition(intakeSubsystem, elevatorSubsystem));
    }
}