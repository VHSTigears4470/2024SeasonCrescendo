package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.differential.ArcadeDrive;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.commands.drivebase.AbsoluteDriveWithFocus;

import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
  // INIT SUBSYSTEMS
  private static final SwerveSubsystem swerveSub = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "neo/swerve"));
  private static final DifferentialSubsystem differentialSub = new DifferentialSubsystem();

  // INIT XBOX CONTROLLER
  public static CommandXboxController xbox1 = new CommandXboxController(0);

  // SMARTDASHBOARD
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // SHUFFLEBOARD
  private ShuffleboardTab shuffleDriverTab = Shuffleboard.getTab("Driver's Tab");

  // Create event map for Path Planner (there should only be one)
  public static final HashMap<String, Command> eventMap = new HashMap<>();

  public RobotContainer() {
    // Configure default commands
    setDriveMode();

    // Configure controller bindings
    configureButtonBindings();

    // Configure auto
    String[] autoList = { "Do Nothing" };
    SmartDashboard.putStringArray("Auto List", autoList);
    initializeAutoChooser();

    // Initialize path planner event maps
    initializeEventMap();
  }

  // update shuffleboard layout
  public void updateShuffleboard() {
  }

  public void initializeAutoChooser() {
    // with command chooser
    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    autoChooser = AutoBuilder.buildAutoChooser();
    shuffleDriverTab.add("Auto Routine", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  public void initializeEventMap() {
    eventMap.put("marker1", new PrintCommand("Pressed Marker 1"));
  }

  // assign button functions
  private void configureButtonBindings() {
    xbox1.a().onTrue(new AbsoluteDriveWithFocus(swerveSub,
        () -> MathUtil.applyDeadband(-xbox1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-xbox1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), "cone"));

  }

  public Command getAutoInput() {
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    if (RobotBase.isReal()) {
      swerveSub.setDefaultCommand(new AbsoluteDrive(swerveSub,
          () -> MathUtil.applyDeadband(-xbox1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(-xbox1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
          () -> MathUtil.applyDeadband(-xbox1.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
          () -> MathUtil.applyDeadband(-xbox1.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND)));
      differentialSub.setDefaultCommand(new ArcadeDrive(differentialSub, xbox1));
    } else {
      swerveSub.setDefaultCommand(swerveSub.simDriveCommand(
          () -> MathUtil.applyDeadband(-xbox1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(-xbox1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
          () -> -xbox1.getRawAxis(2)));
    }
  }

  public void setMotorBrake(boolean brake) {
    swerveSub.setMotorBrake(brake);
  }
}