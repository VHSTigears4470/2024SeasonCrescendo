package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.DeadbandCommandXboxController;
import frc.robot.Constants.DifferentialConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotContainerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.commands.command_groups.*;
import frc.robot.commands.differential.ArcadeDrive;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.commands.drivebase.AbsoluteDriveWithFocus;
import frc.robot.commands.elevator.ElevatorChangePositionIgnoreSoftLimit;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.intake.IntakeSetAmpVoltage;
import frc.robot.commands.intake.IntakeSetIntakeVoltage;
import frc.robot.commands.intake.IntakeSetIntakeVoltageEndWithBreakbeam;
import frc.robot.commands.intake.IntakeSetSpeakerVoltage;
import frc.robot.commands.intake.IntakeSetZeroVoltage;

import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  // INIT SUBSYSTEMS
  private static SwerveSubsystem swerveSub;
  private static DifferentialSubsystem differentialSub;
  private static IntakeSubsystem intakeSub;
  private static ElevatorSubsystem elevatorSub;

  // INIT XBOX CONTROLLER
  public static DeadbandCommandXboxController xbox1;
  public static DeadbandCommandXboxController xbox2;

  // SMARTDASHBOARD
  private SendableChooser<Command> autoChooser;

  // SHUFFLEBOARD
  private ShuffleboardTab shuffleDriverTab;

  // Create event map for Path Planner (there should only be one)
  public static final HashMap<String, Command> eventMap = new HashMap<>();

  public RobotContainer() {

    initializeOtherVars();

    // Initialize drive system (swerve or differential)
    initializeDriveMode();

    // Configure default commands

    configureButtonBindings();

    // Initialize Shuffleboard
    initializeShuffleboard();

    // Initialize path planner command names
    initializeCommandNames();

    // Configure auto
    initializeAutoChooser();
  }

  public void initializeDriveMode() {
    if (DifferentialConstants.USING_DIFFERENTIAL) {
      differentialSub = new DifferentialSubsystem();
    } else {
      differentialSub = null;
    }

    if (SwerveConstants.USING_SWERVE) {
      swerveSub = new SwerveSubsystem(
          new File(Filesystem.getDeployDirectory(), "neo/swerve"));

      if (RobotBase.isReal()) {
        swerveSub.setDefaultCommand(new AbsoluteDrive(swerveSub,
            () -> -xbox1.getLeftY(),
            () -> -xbox1.getLeftX(),
            () -> -xbox1.getRightX(),
            () -> -xbox1.getRightY()));
        // differentialSub.setDefaultCommand(new ArcadeDrive(differentialSub, xbox1));
      } else {
        swerveSub.setDefaultCommand(swerveSub.simDriveCommand(
            () -> -xbox1.getLeftY(),
            () -> -xbox1.getLeftX(),
            () -> -xbox1.getRightX(),
            () -> -xbox1.getRightY()));

        // swerveSub.setDefaultCommand(new AbsoluteDrive(swerveSub,
        // () -> MathUtil.applyDeadband(-xbox1.getLeftY(),
        // OperatorConstants.LEFT_Y_DEADBAND),
        // () -> MathUtil.applyDeadband(-xbox1.getLeftX(),
        // OperatorConstants.LEFT_X_DEADBAND),
        // () -> MathUtil.applyDeadband(-xbox1.getRightX(),
        // OperatorConstants.RIGHT_X_DEADBAND),
        // () -> MathUtil.applyDeadband(-xbox1.getRightY(),
        // OperatorConstants.RIGHT_Y_DEADBAND)));
        // differentialSub.setDefaultCommand(new ArcadeDrive(differentialSub, xbox1));
      }
    } else {
      swerveSub = null;
    }
  }

  public void initializeOtherVars() {
    if (IntakeConstants.IS_USING_INTAKE) {
      intakeSub = new IntakeSubsystem();
    }
    if (ElevatorConstants.IS_USING_ELEVATOR) {
      elevatorSub = new ElevatorSubsystem();
    }
    xbox1 = new DeadbandCommandXboxController(RobotContainerConstants.XBOX_1_ID,
        RobotContainerConstants.XBOX_1_DEADBAND);
    xbox2 = new DeadbandCommandXboxController(RobotContainerConstants.XBOX_2_ID,
        RobotContainerConstants.XBOX_2_DEADBAND);
  }

  public void initializeAutoChooser() {
    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("Do nothing", new WaitCommand(0));
    // Add Pathplanner autos
    autoChooser.addOption("New Auto", swerveSub.getAutonomousCommand("New Auto"));

    shuffleDriverTab.add("Auto Routine", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  @SuppressWarnings("unused")
  public void initializeCommandNames() {
    NamedCommands.registerCommand("test1", new PrintCommand("Test 1 Triggered"));
    if (Constants.IntakeConstants.IS_USING_INTAKE && Constants.ElevatorConstants.IS_USING_ELEVATOR) {
      NamedCommands.registerCommand("Climb Position", new ClimbPosition(intakeSub, elevatorSub));
      NamedCommands.registerCommand("Default Position", new DefaultPosition(intakeSub, elevatorSub));
      NamedCommands.registerCommand("Intake Note", new IntakePositionAndSuck(intakeSub, elevatorSub));
      NamedCommands.registerCommand("Robot Climb", new RobotClimbCommandGroup(intakeSub, elevatorSub));
      NamedCommands.registerCommand("Shoot Amp", new ShootAmpAndReset(intakeSub, elevatorSub));
      NamedCommands.registerCommand("Shoot Speaker", new ShootSpeakerAndReset(intakeSub, elevatorSub));
    }
    if (Constants.IntakeConstants.IS_USING_INTAKE && Constants.ElevatorConstants.IS_USING_ELEVATOR
        && Constants.SwerveConstants.USING_SWERVE) {
      NamedCommands.registerCommand("Drive Till Have Note", new DriveTillHaveNote(intakeSub, elevatorSub, swerveSub));
    }
  }

  // assign button functions
  private void configureButtonBindings() {
    if (SwerveConstants.USING_SWERVE) {
      xbox1.a().onTrue(new AbsoluteDriveWithFocus(swerveSub,
          () -> MathUtil.applyDeadband(-xbox1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(-xbox1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), "cone"));
    }

    if (ElevatorConstants.IS_USING_ELEVATOR) {
      xbox2.a().whileTrue(new ElevatorChangePositionIgnoreSoftLimit(elevatorSub, 0.1));
      xbox2.b().whileTrue(new ElevatorChangePositionIgnoreSoftLimit(elevatorSub, -0.1));
      xbox2.rightBumper().whileTrue(new ElevatorSetHeightState(elevatorSub, ELEVATOR_STATE.UP));
      xbox2.leftBumper().whileTrue(new ElevatorSetHeightState(elevatorSub, ELEVATOR_STATE.DOWN));

      if (IntakeConstants.IS_USING_INTAKE) {
        xbox1.y().whileTrue(new DefaultPosition(intakeSub, elevatorSub));
        xbox2.y().whileTrue(new DefaultPosition(intakeSub, elevatorSub));
        xbox1.start().whileTrue(new ClimbPosition(intakeSub, elevatorSub));
        xbox2.start().whileTrue(new ClimbPosition(intakeSub, elevatorSub));

        xbox2.leftTrigger().whileTrue(new ShootAmpAndReset(intakeSub, elevatorSub));
        xbox2.leftTrigger().whileTrue(new ShootSpeakerAndReset(intakeSub, elevatorSub));

        xbox1.x().whileTrue(new IntakePositionAndSuck(intakeSub, elevatorSub));
        xbox2.x().whileTrue(new IntakePositionAndSuck(intakeSub, elevatorSub));
      }
    }
    if (IntakeConstants.IS_USING_INTAKE) {
      // Testing ONLY
      xbox1.leftTrigger().whileTrue(new IntakeSetIntakeVoltage(intakeSub)).onFalse(new IntakeSetZeroVoltage(intakeSub));
      xbox1.rightTrigger().whileTrue(new IntakeSetSpeakerVoltage(intakeSub))
          .onFalse(new IntakeSetZeroVoltage(intakeSub));
      xbox1.rightBumper().whileTrue(new IntakeSetAmpVoltage(intakeSub)).onFalse(new IntakeSetZeroVoltage(intakeSub));
    }
  }

  public Command getAutoInput() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    if (SwerveConstants.USING_SWERVE) {
      swerveSub.setMotorBrake(brake);
    }
  }

  public void initializeShuffleboard() {
    shuffleDriverTab = Shuffleboard.getTab("Driver's Tab");
  }

  public void updateShuffleboard() {
  }
}