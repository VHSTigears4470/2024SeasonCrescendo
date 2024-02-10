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
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DifferentialConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotContainerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.ElevatorConstants.ELEVATOR_STATE;
import frc.robot.commands.differential.ArcadeDrive;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.commands.drivebase.AbsoluteDriveWithFocus;
import frc.robot.commands.shooter.MoveFeeder;
import frc.robot.commands.shooter.MoveFlywheel;

import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
  // INIT SUBSYSTEMS
  private static SwerveSubsystem swerveSub;
  private static DifferentialSubsystem differentialSub;
  private static IntakeSubsystem intakeSub;
  private static ElevatorSubsystem elevatorSub;
  private static ShooterSubsystem shooterSub;

  // INIT XBOX CONTROLLER
  public static CommandXboxController xbox1;

  // SMARTDASHBOARD
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

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

    // Configure auto
    initializeAutoChooser();

    // Initialize path planner event maps
    initializeEventMap();
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

      if (RobotBase.isReal()) {
        swerveSub.setDefaultCommand(new AbsoluteDrive(swerveSub,
            () -> MathUtil.applyDeadband(-xbox1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-xbox1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> MathUtil.applyDeadband(-xbox1.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
            () -> MathUtil.applyDeadband(-xbox1.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND)));
      } else {
        swerveSub.setDefaultCommand(swerveSub.simDriveCommand(
            () -> MathUtil.applyDeadband(-xbox1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-xbox1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> -xbox1.getRawAxis(2)));
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
    xbox1 = new CommandXboxController(RobotContainerConstants.XBOX_1_ID);
    if (ShooterConstants.IS_USING_SHOOTER) {
      shooterSub = new ShooterSubsystem();
    }
  }

  public void initializeAutoChooser() {
    // with command chooser
    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    if (SwerveConstants.USING_SWERVE) {
      autoChooser = AutoBuilder.buildAutoChooser();
    }
    shuffleDriverTab.add("Auto Routine", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  public void initializeEventMap() {
    eventMap.put("marker1", new PrintCommand("Pressed Marker 1"));
  }

  // assign button functions
  private void configureButtonBindings() {
    if (SwerveConstants.USING_SWERVE) {
      xbox1.a().onTrue(new AbsoluteDriveWithFocus(swerveSub,
          () -> MathUtil.applyDeadband(-xbox1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(-xbox1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), "cone"));
    }

    if (ElevatorConstants.IS_USING_ELEVATOR) {
      xbox1.rightTrigger().whileTrue(elevatorSub.changePositionCommandIgnoreSoftLimit(0.1));
      xbox1.leftTrigger().whileTrue(elevatorSub.changePositionCommandIgnoreSoftLimit(-0.1));
      xbox1.a().whileTrue(elevatorSub.setHeightStateCommand(ELEVATOR_STATE.UP));
      xbox1.b().whileTrue(elevatorSub.setHeightStateCommand(ELEVATOR_STATE.DOWN));
    }

    if (ShooterConstants.IS_USING_SHOOTER) {
      xbox1.leftBumper().whileTrue(new MoveFeeder(shooterSub, ShooterConstants.FEEDER_IN_VOLTAGE));
      xbox1.leftTrigger().whileTrue(new MoveFeeder(shooterSub, ShooterConstants.FEEDER_OUT_VOLTAGE));
      xbox1.rightBumper().whileTrue(new MoveFlywheel(shooterSub, ShooterConstants.FLYWHEEL_IN_VOLTAGE));
      xbox1.rightTrigger().whileTrue(new MoveFlywheel(shooterSub, ShooterConstants.FLYWHEEL_OUT_VOLTAGE));
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