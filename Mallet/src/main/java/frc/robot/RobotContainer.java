package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
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
import frc.robot.subsystems.PoseEstimation;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.DeadbandCommandXboxController;
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DifferentialConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.NoteLLConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.commands.command_groups.*;
import frc.robot.commands.differential.ArcadeDrive;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.commands.drivebase.AbsoluteDriveWithFocus;
import frc.robot.commands.drivebase.ResetOdom;
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
import frc.robot.commands.intake.IntakeSetIntakeVoltageEndWithBreakbeam;
import frc.robot.commands.intake.IntakeSetSlowShoot;
import frc.robot.commands.intake.IntakeSetSpeakerVoltage;
import frc.robot.commands.intake.IntakeSetZeroVoltage;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  // INIT SUBSYSTEMS
  private static SwerveSubsystem swerveSub;
  private static DifferentialSubsystem differentialSub;
  private static IntakeSubsystem intakeSub;
  private static ElevatorSubsystem elevatorSub;
  // private static PhotonSubsystem photonSub;
  private static PoseEstimation poseEstimate;
  private static NoteLimelight limelightSub;

  // INIT XBOX CONTROLLER
  public static DeadbandCommandXboxController xbox1; // Driver
  public static DeadbandCommandXboxController xbox2; // Operator

  // SMARTDASHBOARD
  private SendableChooser<Command> autoPresetChooser;

  // private GenericEntry
  private SendableChooser<String> basePositionChooser;
  private ArrayList<SendableChooser<String>> autoDirections; // List to hold all the auto directions set by the driver

  // SHUFFLEBOARD
  private ShuffleboardTab shuffleDriverTab;
  private ShuffleboardTab shuffleDebugTab;
  private ShuffleboardLayout shuffleDebugElevatorCommandList;
  private ShuffleboardLayout shuffleDebugIntakeCommandList;

  // Create event map for Path Planner (there should only be one)
  public static final HashMap<String, Command> eventMap = new HashMap<>();

  public RobotContainer() {
    // Initialize the other subsystems and controllers
    initializeOtherVars();
    // Initialize drive system (swerve or differential)
    initializeDriveMode();

    // Initialize Shuffleboard
    initializeShuffleboard();

    // Configure default commands
    configureButtonBindings();

    // Initialize path planner command names
    initializeCommandNames();

    // Configure auto
    initializeAutoChooser();
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
          // swerveSub.setDefaultCommand(swerveSub.simDriveCommand(
          // () -> -xbox1.getLeftY(),
          // () -> -xbox1.getLeftX(),
          // () -> -xbox1.getRawAxis(2)));
          swerveSub.setDefaultCommand(swerveSub.simDriveCommand(
              () -> -xbox1.getLeftY(),
              () -> -xbox1.getLeftX(),
              () -> -xbox1.getRightX(),
              () -> -xbox1.getRightY()));
        }

        // poseEstimate = new PoseEstimation(swerveSub);
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
    // Init Auto Chooser
    basePositionChooser = new SendableChooser<String>();
    autoDirections = new ArrayList<SendableChooser<String>>();
    autoPresetChooser = new SendableChooser<Command>();

    // Base starting position and position to shoot into speaker with
    basePositionChooser.setDefaultOption("Amp Side", AutoConstants.AMP_SIDE_START);
    basePositionChooser.addOption("Middle Side", AutoConstants.MIDDLE_SIDE_START);
    basePositionChooser.addOption("Feeder Side", AutoConstants.FEEDER_SIDE_START);

    // Fills with number of wanted directions
    for (int i = 0; i < AutoConstants.numOfDirections; i++) {
      autoDirections.add(new SendableChooser<String>());
    }

    // Fills each Combo Chooser with all options
    for (SendableChooser<String> compiledCommandEnd : autoDirections) {
      compiledCommandEnd.setDefaultOption("Do Nothing", "Nothing"); // Depends on whether at the speaker or not
      if (SwerveConstants.USING_SWERVE) {
        compiledCommandEnd.addOption("Amp Side", AutoConstants.AMP_SIDE_START);
        compiledCommandEnd.addOption("Middle Side", AutoConstants.MIDDLE_SIDE_START);
        compiledCommandEnd.addOption("Feeder Side", AutoConstants.FEEDER_SIDE_START);

        compiledCommandEnd.addOption("Amp Wing Cycle", AutoConstants.AMP_WING_NOTE_ENDING);
        compiledCommandEnd.addOption("Middle Wing Cycle", AutoConstants.MIDDLE_WING_NOTE_ENDING);
        compiledCommandEnd.addOption("Feeder Wing Cycle", AutoConstants.FEEDER_WING_NOTE_ENDING);

        compiledCommandEnd.addOption("Amp Center Note", AutoConstants.AMP_CENTER_NOTE_ENDING);
        compiledCommandEnd.addOption("Amp Middle Center Note", AutoConstants.AMP_MIDDLE_CENTER_NOTE_ENDING);
        compiledCommandEnd.addOption("Middle Center Note", AutoConstants.MIDDLE_CENTER_NOTE_ENDING);
        compiledCommandEnd.addOption("Feeder Middle Center Note", AutoConstants.FEEDER_MIDDLE_CENTER_NOTE_ENDING);
        compiledCommandEnd.addOption("Feeder Center Note", AutoConstants.FEEDER_CENTER_NOTE_ENDING);
      }

      // Named commands
      if (IntakeConstants.IS_USING_INTAKE) {
        compiledCommandEnd.addOption("Shoot speaker", "Shoot Speaker");
        compiledCommandEnd.addOption("Shoot amp", "Shoot Amp");

        compiledCommandEnd.addOption("Drive Til Have Note", AutoConstants.DRIVE_TIL_HAVE_NOTE);
      }
    }
    // Adds the base position to the shuffleboard list
    shuffleDriverTab.getLayout("Directions", BuiltInLayouts.kList)
        .add("Base Position", basePositionChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);

    // Adds each number of directions to the shuffleboard list
    for (int i = 0; i < autoDirections.size(); i++) {
      shuffleDriverTab.getLayout("Directions", BuiltInLayouts.kList)
          .add("#" + (i + 1) + " Note", autoDirections.get(i))
          .withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    // Init auto preset chooser
    autoPresetChooser.setDefaultOption("Use Modular", null);
    if (SwerveConstants.USING_SWERVE && ElevatorConstants.IS_USING_ELEVATOR && IntakeConstants.IS_USING_INTAKE) {
      if (SwerveConstants.USING_SWERVE) {
        autoPresetChooser.addOption("TEST ONLY STRAIGHT", new ResetOdom(swerveSub).andThen(
            swerveSub.driveToPose(swerveSub.createPose(1, 0))));
        autoPresetChooser.addOption("TEST ONLY SIDE", new ResetOdom(swerveSub).andThen(
            swerveSub.driveToPose(swerveSub.createPose(1, 1))));
        autoPresetChooser.addOption("TEST ONLY DIAG", new ResetOdom(swerveSub).andThen(
            swerveSub.driveToPose(swerveSub.createPose(0, 1))));
      }
      autoPresetChooser.addOption("Preset One (Amp Side)",
          new ShootSpeakerAndReset(intakeSub, elevatorSub)
              .andThen(swerveSub.getAutonomousCommand(
                  AutoConstants.AMP_SIDE_START + " to " + AutoConstants.AMP_WING_NOTE_ENDING, false))
              .andThen(new DriveTillHaveNote(intakeSub, elevatorSub, swerveSub))
              .andThen(swerveSub.getAutonomousCommand(
                  AutoConstants.AMP_WING_NOTE_ENDING + " to " + AutoConstants.AMP_SIDE_START, false))
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub))
              .andThen(swerveSub.getAutonomousCommand(
                  AutoConstants.AMP_SIDE_START + " to " + AutoConstants.AMP_CENTER_NOTE_ENDING, false)));
      autoPresetChooser.addOption("Preset Two (Middle Side)",
          new ShootSpeakerAndReset(intakeSub, elevatorSub)
              .andThen(swerveSub.getAutonomousCommand(
                  AutoConstants.MIDDLE_SIDE_START + " to " + AutoConstants.MIDDLE_WING_NOTE_ENDING, false))
              .andThen(new DriveTillHaveNote(intakeSub, elevatorSub, swerveSub))
              .andThen(swerveSub.getAutonomousCommand(
                  AutoConstants.MIDDLE_WING_NOTE_ENDING + " to " + AutoConstants.MIDDLE_SIDE_START, false))
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub))
              .andThen(swerveSub.getAutonomousCommand(
                  AutoConstants.MIDDLE_SIDE_START + " to " + AutoConstants.MIDDLE_CENTER_NOTE_ENDING, false)));
      autoPresetChooser.addOption("Preset Three (Feeder Side)",
          new ShootSpeakerAndReset(intakeSub, elevatorSub)
              .andThen(swerveSub.getAutonomousCommand(
                  AutoConstants.FEEDER_SIDE_START + " to " + AutoConstants.FEEDER_WING_NOTE_ENDING, false))
              .andThen(new DriveTillHaveNote(intakeSub, elevatorSub, swerveSub))
              .andThen(swerveSub.getAutonomousCommand(
                  AutoConstants.FEEDER_WING_NOTE_ENDING + " to " + AutoConstants.FEEDER_SIDE_START, false))
              .andThen(new ShootSpeakerAndReset(intakeSub, elevatorSub))
              .andThen(swerveSub.getAutonomousCommand(
                  AutoConstants.FEEDER_SIDE_START + " to " + AutoConstants.FEEDER_CENTER_NOTE_ENDING, false)));
    }
    shuffleDebugTab.add("Presets", autoPresetChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  public void initializeCommandNames() {
    if (Constants.IntakeConstants.IS_USING_INTAKE && Constants.ElevatorConstants.IS_USING_ELEVATOR) {
      NamedCommands.registerCommand(AutoConstants.CLIMB_POSITION, new ClimbPosition(intakeSub, elevatorSub));
      NamedCommands.registerCommand(AutoConstants.DEFAULT_POSITION, new DefaultPosition(intakeSub, elevatorSub));
      NamedCommands.registerCommand(AutoConstants.INTAKE_POSITION, new IntakePosition(intakeSub, elevatorSub));

      NamedCommands.registerCommand(AutoConstants.INTAKE_NOTE, new IntakePositionAndSuck(intakeSub, elevatorSub));
      NamedCommands.registerCommand(AutoConstants.SHOOT_AMP, new ShootAmpAndReset(intakeSub, elevatorSub));
      NamedCommands.registerCommand(AutoConstants.SHOOT_SPEAKER, new ShootSpeakerAndReset(intakeSub, elevatorSub));
    }
    if (Constants.IntakeConstants.IS_USING_INTAKE && Constants.ElevatorConstants.IS_USING_ELEVATOR
        && Constants.SwerveConstants.USING_SWERVE) {
      NamedCommands.registerCommand(AutoConstants.DRIVE_TIL_HAVE_NOTE,
          new DriveTillHaveNote(intakeSub, elevatorSub, swerveSub));
    }
  }

  // assign button functions
  private void configureButtonBindings() {
    // XBOX 1 Configs
    if (OperatorConstants.USING_XBOX_1) {
      if (SwerveConstants.USING_SWERVE && NoteLLConstants.IS_USING_NOTE_LIMELIGHT) {
        xbox1.a().onTrue(new AbsoluteDriveWithFocus(swerveSub, limelightSub,
            () -> -xbox1.getLeftY(),
            () -> -xbox1.getLeftX()));
      }

      if (IntakeConstants.IS_USING_INTAKE && SwerveConstants.USING_SWERVE && ElevatorConstants.IS_USING_ELEVATOR
          && NoteLLConstants.IS_USING_NOTE_LIMELIGHT) {
        xbox1.y().whileTrue(new IntakeNoteWhileFocus(swerveSub, intakeSub, elevatorSub, limelightSub)).onFalse(
            new IntakeSetZeroVoltage(intakeSub));
      }

      if (IntakeConstants.IS_USING_INTAKE && ElevatorConstants.IS_USING_ELEVATOR) {
        // xbox1.y().whileTrue(new DefaultPosition(intakeSub, elevatorSub));
        xbox1.start().whileTrue(new ClimbPosition(intakeSub, elevatorSub));
        // xbox1.x().whileTrue(new IntakePositionAndSuck(intakeSub, elevatorSub));
      }

      if (IntakeConstants.IS_USING_INTAKE && IntakeConstants.DEBUG) {
        // TODO - Remove the debug commands for real testing
        // Testing ONLY
        // xbox1.leftBumper().whileTrue(new IntakeSetIntakeVoltage(intakeSub))
        // .onFalse(new IntakeSetZeroVoltage(intakeSub));
        // xbox1.rightBumper().whileTrue(new IntakeSetSpeakerVoltage(intakeSub))
        // .onFalse(new IntakeSetZeroVoltage(intakeSub));
        // xbox1.a().whileTrue(new IntakeSetAmpVoltage(intakeSub))
        // .onFalse(new IntakeSetZeroVoltage(intakeSub));

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
        // xbox1.leftTrigger().whileTrue(new ElevatorChangePosition(elevatorSub,
        // -0.15));
        // xbox1.rightTrigger().whileTrue(new ElevatorChangePosition(elevatorSub,
        // 0.15));
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

      }

      if (IntakeConstants.IS_USING_INTAKE && IntakeConstants.DEBUG && ElevatorConstants.IS_USING_ELEVATOR
          && ElevatorConstants.DEBUG) {
        // Positions
        shuffleDebugElevatorCommandList.add("Default Full Position", new DefaultPosition(intakeSub, elevatorSub))
            .withWidget(BuiltInWidgets.kCommand);
        shuffleDebugElevatorCommandList.add("Climb Full Position", new ClimbPosition(intakeSub, elevatorSub))
            .withWidget(BuiltInWidgets.kCommand);
        shuffleDebugElevatorCommandList.add("IntakePosition", new IntakePosition(intakeSub, elevatorSub))
            .withWidget(BuiltInWidgets.kCommand);
        shuffleDebugElevatorCommandList.add("IntakeAndSuckPosition", new IntakePositionAndSuck(intakeSub, elevatorSub))
            .withWidget(BuiltInWidgets.kCommand);
        shuffleDebugElevatorCommandList.add("ShootAmp&Default", new ShootAmpAndReset(intakeSub, elevatorSub))
            .withWidget(BuiltInWidgets.kCommand);
        shuffleDebugElevatorCommandList.add("ShootSpeaker&Default", new ShootSpeakerAndReset(intakeSub, elevatorSub))
            .withWidget(BuiltInWidgets.kCommand);

      }
    }
    // XBOX 2 Configs
    if (OperatorConstants.USING_XBOX_2) {
      if (IntakeConstants.IS_USING_INTAKE && ElevatorConstants.IS_USING_ELEVATOR && SwerveConstants.USING_SWERVE
          && NoteLLConstants.IS_USING_NOTE_LIMELIGHT) {
        xbox2.rightStick().whileTrue(new IntakeNoteWhileFocus(swerveSub, intakeSub, elevatorSub, limelightSub)).onFalse(
            new IntakeSetZeroVoltage(intakeSub));
      }
      if (IntakeConstants.IS_USING_INTAKE && ElevatorConstants.IS_USING_ELEVATOR) {
        xbox2.b().onTrue(new ShootSpeakerAndReset(intakeSub, elevatorSub));
        xbox2.leftStick().onTrue(new ShootAmpPosition(intakeSub, elevatorSub));
        xbox2.x().whileTrue(new RevUpAndShootAmp(intakeSub, elevatorSub)).onFalse(
            new IntakeSetZeroVoltage(intakeSub));
        xbox2.a().onTrue(new DefaultPosition(intakeSub, elevatorSub));
        xbox2.y().onTrue(new IntakePositionAndSuck(intakeSub, elevatorSub));
        xbox2.start().onTrue(new ClimbPosition(intakeSub, elevatorSub));
        xbox2.leftTrigger().whileTrue(new ElevatorChangePosition(elevatorSub,
            -0.15));
        xbox2.rightTrigger().whileTrue(new ElevatorChangePosition(elevatorSub,
            0.15));
        xbox2.povDown().whileTrue(new ElevatorChangePositionIgnoreSoftLimit(elevatorSub,
            -0.15));
        xbox2.povUp().whileTrue(new ElevatorChangePositionIgnoreSoftLimit(elevatorSub,
            0.15));
      }
      if (IntakeConstants.IS_USING_INTAKE) {
        xbox2.rightBumper().whileTrue(new IntakeShootSlow(intakeSub)).onFalse(
            new IntakeSetZeroVoltage(intakeSub));
        xbox2.leftBumper().whileTrue(new IntakeSetIntakeVoltageEndWithBreakbeam(intakeSub)).onFalse(
            new IntakeSetZeroVoltage(intakeSub));

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
      }
    }
  }

  public Command getAutonomousCommand() {
    // If the user did not press "Use Modular", then return the preset command they
    // selected
    if (autoPresetChooser.getSelected() != null) {
      return autoPresetChooser.getSelected();
    }
    // Object to store sequential command. This is done to init a command that does
    // nothing (at first)
    SequentialCommandGroup compiledCommand = new SequentialCommandGroup();

    // Where the robot starts at each direction
    String startPos = basePositionChooser.getSelected();

    for (SendableChooser<String> choice : autoDirections) {
      // Default choosen command does nothing
      Command choosenCommand = null;
      // Selected choice as a string
      String selected = choice.getSelected();

      // If command is not a path command but a normal named command
      Command namedCommand = NamedCommands.getCommand(selected);
      // If named command
      if (namedCommand != null) {
        choosenCommand = namedCommand;
      }
      // If path planner command
      else {
        Command pathPlannerComand = swerveSub.getAutonomousCommand(startPos + " to " + selected, false);
        if (pathPlannerComand != null) {
          choosenCommand = pathPlannerComand;
        }
        startPos = selected;
      }

      // Apends it to the sequential command
      if (choosenCommand == null || selected.equals("Nothing")) {
        continue;
      }
      compiledCommand = compiledCommand.andThen(choosenCommand);
    }

    // returns the compiled sequential command
    return compiledCommand;
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