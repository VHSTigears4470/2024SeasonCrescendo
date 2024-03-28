package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CycleTimes;
import frc.robot.commands.intake.IntakePusherExtend;
import frc.robot.commands.intake.IntakeSetIntakeVoltageEndWithBreakbeam;
import frc.robot.commands.intake.IntakeSetSlowShoot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeShootSlow extends SequentialCommandGroup {
    public IntakeShootSlow(IntakeSubsystem intakeSubsystem) {
        addRequirements(intakeSubsystem);
        // Start if there is no note in the intake
        addCommands(
                new IntakeSetSlowShoot(intakeSubsystem),
                new WaitCommand(CycleTimes.INTAKE_MOTORS_WARM_UP_CYCLE_TIME),
                new IntakePusherExtend(intakeSubsystem));

    }
}