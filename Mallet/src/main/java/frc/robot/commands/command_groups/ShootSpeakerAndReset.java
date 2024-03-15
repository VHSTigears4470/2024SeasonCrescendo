package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakePusherExtend;
import frc.robot.Constants.CycleTimes;
import frc.robot.commands.intake.IntakePositionRetract;
import frc.robot.commands.intake.IntakeSetSpeakerVoltage;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootSpeakerAndReset extends SequentialCommandGroup {
    public ShootSpeakerAndReset(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addRequirements(intakeSubsystem, elevatorSubsystem);
        // Start if note is in the intake
        if (intakeSubsystem.getNoteBreakbeam()) {
            addCommands(
                    // TODO: Light on the robot glows when in correct range/radius around the
                    // speaker and pointing in correct direction
                    new IntakePositionRetract(intakeSubsystem),
                    new WaitCommand(CycleTimes.INTAKE_MOTORS_PUSHER_CYCLE_TIME),
                    new IntakeSetSpeakerVoltage(intakeSubsystem),
                    new WaitCommand(CycleTimes.INTAKE_MOTORS_WARM_UP_CYCLE_TIME),
                    new IntakePusherExtend(intakeSubsystem),
                    new WaitCommand(CycleTimes.INTAKE_MOTORS_WARM_UP_CYCLE_TIME),
                    new DefaultPosition(intakeSubsystem, elevatorSubsystem));
        }
    }
}