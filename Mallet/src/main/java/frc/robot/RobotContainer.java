package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteLimelight;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DeadbandCommandXboxController;
import frc.robot.Constants.DifferentialConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.NoteLLConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.commands.command_groups.*;
import frc.robot.commands.differential.ArcadeDrive;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.commands.drivebase.AbsoluteDriveWithFocus;
import frc.robot.commands.elevator.ElevatorChangePosition;
import frc.robot.commands.elevator.ElevatorChangePositionIgnoreSoftLimit;
import frc.robot.commands.elevator.ElevatorSetHeightState;
import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.intake.IntakePositionDown;
import frc.robot.commands.intake.IntakePositionUp;
import frc.robot.commands.intake.IntakePusherExtend;
import frc.robot.commands.intake.IntakePusherRetract;
import frc.robot.commands.intake.IntakeSetAmpVoltage;
import frc.robot.commands.intake.IntakeSetIntakeVoltage;
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
  private static PhotonSubsystem photonSub;
  private static NoteLimelight limelightSub;

  // INIT XBOX CONTROLLER
  public static DeadbandCommandXboxController xbox1; // Driver
  public static DeadbandCommandXboxController xbox2; // Operator

  // SMARTDASHBOARD
  private SendableChooser<Command> autoChooser;

  // SHUFFLEBOARD
  private ShuffleboardTab shuffleDriverTab;
  private ShuffleboardTab shuffleDebugTab;
  private ShuffleboardLayout shuffleDebugElevatorCommandList;
  private ShuffleboardLayout shuffleDebugIntakeCommandList;

  // Create event map for Path Planner (there should only be one)
  public static final HashMap<String, Command> eventMap = new HashMap<>();

  public RobotContainer() {

    initializeOtherVars();

    // Initialize drive system (swerve or differential)
    initializeDriveMode();
    // Initialize the other subsystems and controllers
    initializeOtherVars();

    // Initialize Shuffleboard
    initializeShuffleboard();

    // Configure default commands
    configureButtonBindings();

    // Initialize path planner command names
    // initializeCommandNames();
    // TODO: Reenable

    // Configure auto
    // initializeAutoChooser();
  }

  public void initializeDriveMode() {
    if (DifferentialConstants.USING_DIFFERENTIAL) {
      differentialSub = new DifferentialSubsystem();
      differentialSub.setDefaultCommand(new ArcadeDrive(differentialSub, xbox1));
    } else {
      differentialSub = null;
    }

    if (SwerveConstants.USING_SWERVE) {
      swerveSub = new SwerveSubsystem(
          new File(Filesystem.getDeployDirectory(), "neo/swerve"));

      if (OperatorConstants.USING_XBOX_1) {
        if (RobotBase.isReal()) {
          swerveSub.setDefaultCommand(new AbsoluteDrive(swerveSub,
              () -> -xbox1.getLeftY(),
              () -> -xbox1.getLeftX(),
              () -> -xbox1.getRightX(),
              () -> -xbox1.getRightY()));
        } else {
          swerveSub.setDefaultCommand(swerveSub.simDriveCommand(
              () -> -xbox1.getLeftY(),
              () -> -xbox1.getLeftX(),
              () -> -xbox1.getRawAxis(2)));
        }
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
    if (PhotonConstants.USING_VISION) {
      photonSub = new PhotonSubsystem();
      // Set up vision readings for Swerve
      if (SwerveConstants.USING_SWERVE) {
        swerveSub.setupVisionMeasurement(
            () -> {
              return photonSub.getEstimatedRobotPoseFromLeftPhoton(swerveSub.getPose());
            },
            () -> {
              return photonSub.getEstimatedRobotPoseFromLeftPhoton(swerveSub.getPose());
            });
      }
    }
    if (NoteLLConstants.IS_USING_NOTE_LIMELIGHT) {
      limelightSub = new NoteLimelight();
    }
    if (OperatorConstants.USING_XBOX_1) {
      xbox1 = new DeadbandCommandXboxController(OperatorConstants.XBOX_1_ID,
      OperatorConstants.XBOX_1_DEADBAND);
    }
    if (OperatorConstants.USING_XBOX_2) {
      xbox2 = new DeadbandCommandXboxController(OperatorConstants.XBOX_2_ID, OperatorConstants.XBOX_2_DEADBAND);
    }
  }

  public void initializeAutoChooser() {
    // with command chooser
    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    if (SwerveConstants.USING_SWERVE && Constants.IntakeConstants.IS_USING_INTAKE
        && Constants.ElevatorConstants.IS_USING_ELEVATOR) {
      // autoChooser = AutoBuilder.buildAutoChooser();
      // Amp Start
      autoChooser.addOption("Amp Start to Amp Wing Cycle",
          swerveSub.getAutonomousCommand("Amp Start to Amp Wing Cycle", false)
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub)));
      autoChooser.addOption("Amp Start to Middle Wing Cycle",
          swerveSub.getAutonomousCommand("Amp Start to Middle Wing Cycle", false)
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub)));
      autoChooser.addOption("Amp Start to Feeder Wing Cycle",
          swerveSub.getAutonomousCommand("Amp Start to Feeder Wing Cycle", false)
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub)));
      // Middle Start
      autoChooser.addOption("Middle Start to Feeder Wing Cycle",
          swerveSub.getAutonomousCommand("Middle Start to Feeder Wing Cycle", false)
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub)));
      autoChooser.addOption("Middle Start to Amp Wing Cycle",
          swerveSub.getAutonomousCommand("Middle Start to Middle Wing Cycle", false)
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub)));
      autoChooser.addOption("Middle Start to Amp Wing Cycle",
          swerveSub.getAutonomousCommand("Middle Start to Amp Wing Cycle", false)
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub)));
      // Feeder Start
      autoChooser.addOption("Feeder Start to Feeder Wing Cycle",
          swerveSub.getAutonomousCommand("Feeder Start to Feeder Wing Cycle", false)
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub)));
      autoChooser.addOption("Feeder Start to Middle Wing Cycle",
          swerveSub.getAutonomousCommand("Feeder Start to Middle Wing Cycle", false)
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub)));
      autoChooser.addOption("Feeder Start to Amp Wing Cycle",
          swerveSub.getAutonomousCommand("Feeder Start to Amp Wing Cycle", false)
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub)));
    }
    shuffleDriverTab.add("Auto Routine", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  public void initializeCommandNames() {
    NamedCommands.registerCommand("test1", new PrintCommand("Test 1 Triggered"));
    if (Constants.IntakeConstants.IS_USING_INTAKE && Constants.ElevatorConstants.IS_USING_ELEVATOR) {
      NamedCommands.registerCommand("Climb Position", new ClimbPosition(intakeSub, elevatorSub));
      NamedCommands.registerCommand("Default Position", new DefaultPosition(intakeSub, elevatorSub));
      NamedCommands.registerCommand("Intake Note", new IntakePositionAndSuck(intakeSub, elevatorSub));
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
    // XBOX 1 Configs
    if (OperatorConstants.USING_XBOX_1) {
      if (SwerveConstants.USING_SWERVE) {
        xbox1.a().onTrue(new AbsoluteDriveWithFocus(swerveSub, limelightSub,
            () -> -xbox1.getLeftY(),
            () -> -xbox1.getLeftX()));
      }

      // if (IntakeConstants.IS_USING_INTAKE && ElevatorConstants.IS_USING_ELEVATOR) {
      // xbox1.y().whileTrue(new DefaultPosition(intakeSub, elevatorSub));
      // xbox1.start().whileTrue(new ClimbPosition(intakeSub, elevatorSub));
      // xbox1.x().whileTrue(new IntakePositionAndSuck(intakeSub, elevatorSub));
      // }

      if (IntakeConstants.IS_USING_INTAKE && IntakeConstants.DEBUG) {
        // TODO - Remove the debug commands for real testing
        // Testing ONLY
        xbox1.leftBumper().whileTrue(new IntakeSetIntakeVoltage(intakeSub))
            .onFalse(new IntakeSetZeroVoltage(intakeSub));
        xbox1.rightBumper().whileTrue(new IntakeSetSpeakerVoltage(intakeSub))
            .onFalse(new IntakeSetZeroVoltage(intakeSub));
        xbox1.a().whileTrue(new IntakeSetAmpVoltage(intakeSub))
            .onFalse(new IntakeSetZeroVoltage(intakeSub));

        shuffleDebugIntakeCommandList.add("Intake Retract", new IntakePositionUp(intakeSub))
            .withWidget(BuiltInWidgets.kCommand);
        shuffleDebugIntakeCommandList.add("Intake Extend", new IntakePositionDown(intakeSub))
            .withWidget(BuiltInWidgets.kCommand);
        shuffleDebugIntakeCommandList.add("Intake Pusher Retract", new IntakePusherRetract(intakeSub))
            .withWidget(BuiltInWidgets.kCommand);
        shuffleDebugIntakeCommandList.add("Intake Pusher Extend", new IntakePusherExtend(intakeSub))
            .withWidget(BuiltInWidgets.kCommand);
      }

      if (ElevatorConstants.IS_USING_ELEVATOR && ElevatorConstants.DEBUG) {
        // TODO - Remove the debug commands for real testing
        xbox1.leftTrigger().whileTrue(new ElevatorChangePosition(elevatorSub, -0.15));
        xbox1.rightTrigger().whileTrue(new ElevatorChangePosition(elevatorSub, 0.15));
        shuffleDebugElevatorCommandList.add("Elevator Up No Soft Limit",
            new ElevatorChangePositionIgnoreSoftLimit(elevatorSub, 0.15)).withWidget(BuiltInWidgets.kCommand);
        shuffleDebugElevatorCommandList.add("Elevator Down No Soft Limit",
            new ElevatorChangePositionIgnoreSoftLimit(elevatorSub, -0.15)).withWidget(BuiltInWidgets.kCommand);
        shuffleDebugElevatorCommandList.add("Elevator Zero", new ElevatorZero(elevatorSub))
            .withWidget(BuiltInWidgets.kCommand);
        shuffleDebugElevatorCommandList.add("Elevator Up", new ElevatorSetHeightState(elevatorSub, ELEVATOR_STATE.UP))
            .withWidget(BuiltInWidgets.kCommand);
        shuffleDebugElevatorCommandList.add("Elevator Down",
            new ElevatorSetHeightState(elevatorSub, ELEVATOR_STATE.DOWN)).withWidget(BuiltInWidgets.kCommand);
        shuffleDebugElevatorCommandList
            .add("Elevator Climb", new ElevatorSetHeightState(elevatorSub, ELEVATOR_STATE.CLIMB))
            .withWidget(BuiltInWidgets.kCommand);

        // Positions
        shuffleDebugElevatorCommandList.add("Default Position", new DefaultPosition(intakeSub, elevatorSub))
            .withWidget(BuiltInWidgets.kCommand);
        shuffleDebugElevatorCommandList.add("IntakeAndSuckPosition", new IntakePositionAndSuck(intakeSub, elevatorSub))
            .withWidget(BuiltInWidgets.kCommand);
      }
    }
    // XBOX 2 Configs
    if (OperatorConstants.USING_XBOX_2) {
      // if (ElevatorConstants.IS_USING_ELEVATOR && !ElevatorConstants.DEBUG) {
      // xbox2.a().whileTrue(new ElevatorChangePositionIgnoreSoftLimit(elevatorSub,
      // 0.1));
      // xbox2.b().whileTrue(new ElevatorChangePositionIgnoreSoftLimit(elevatorSub,
      // -0.1));

      // xbox2.rightBumper().whileTrue(new ElevatorSetHeightState(elevatorSub,
      // ELEVATOR_STATE.UP));
      // xbox2.leftBumper().whileTrue(new ElevatorSetHeightState(elevatorSub,
      // ELEVATOR_STATE.DOWN));
      // }

      // if (IntakeConstants.IS_USING_INTAKE && ElevatorConstants.IS_USING_ELEVATOR) {
      // xbox2.x().whileTrue(new IntakePositionAndSuck(intakeSub, elevatorSub));
      // xbox2.y().whileTrue(new DefaultPosition(intakeSub, elevatorSub));

      // xbox2.start().whileTrue(new ClimbPosition(intakeSub, elevatorSub));

      // xbox2.leftTrigger().whileTrue(new ShootAmpAndReset(intakeSub, elevatorSub));
      // xbox2.rightTrigger().whileTrue(new ShootSpeakerAndReset(intakeSub,
      // elevatorSub));
      // }
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    if (SwerveConstants.USING_SWERVE) {
      swerveSub.setMotorBrake(brake);
    }
  }

  public void initializeShuffleboard() {
    shuffleDriverTab = Shuffleboard.getTab("Driver's Tab");
    shuffleDebugTab = Shuffleboard.getTab("Debug Tab");
    shuffleDebugElevatorCommandList = shuffleDebugTab.getLayout("Elevator Command List", BuiltInLayouts.kList);
    shuffleDebugIntakeCommandList = shuffleDebugTab.getLayout("Intake Command List", BuiltInLayouts.kList);
  }

  public void updateShuffleboard() {
  }
}