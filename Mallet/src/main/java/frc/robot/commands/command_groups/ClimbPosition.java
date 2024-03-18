package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.CycleTimes;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakePositionRetract;

public class ClimbPosition extends SequentialCommandGroup {
        public ClimbPosition(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
                addRequirements(intakeSubsystem, elevatorSubsystem);
                addCommands(
                                new ParallelCommandGroup(
                                                // TODO: Ask for more clarification on it needs to wait before intaking
                                                // (when it is getting ready to climb)
                                                new SequentialCommandGroup(new WaitCommand(
                                                                CycleTimes.INTAKE_MOTORS_PUSHER_CYCLE_TIME),
                                                                new IntakePositionRetract(intakeSubsystem)),
                                                new ElevatorSetHeightState(elevatorSubsystem,
                                                                ElevatorConstants.ELEVATOR_STATE.CLIMB)));
        }
}
