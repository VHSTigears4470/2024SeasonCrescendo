package frc.robot.commands.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakePositionRetract;

public class ClimbPosition extends SequentialCommandGroup {
    public ClimbPosition(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(new WaitCommand(.7),
                                new IntakePositionRetract(intakeSubsystem)),
                        new ElevatorSetHeightState(elevatorSubsystem, ElevatorConstants.ELEVATOR_STATE.CLIMB)));
    }
 } 
