package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.commands.drivebase.AbsoluteDriveWithFocus;
import edu.wpi.first.wpilibj.I2C;


import java.io.File;
import java.util.HashMap;

public class RobotContainer {
  // INIT SUBSYSTEMS
  private static final Drivetrain m_drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(), "neo/swerve"));
  
  // INIT XBOX CONTROLLER
  public static XboxController m_xbox = new XboxController(0);

  // INIT JOYSTICK ARRAYS
  public static HashMap<String, Trigger> controllerButtons_arm = new HashMap<String, Trigger>();
  public static HashMap<String, Trigger> controllerButtons_drive = new HashMap<String, Trigger>();

  // SMARTDASHBOARD
  // private SendableChooser<String> m_autoChooser = new SendableChooser<String>();
  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  // SHUFFLEBOARD
  private ShuffleboardTab main = Shuffleboard.getTab("Driver's Tab");

  public RobotContainer() {
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(new AbsoluteDrive(m_drivetrain, () -> MathUtil.applyDeadband(-m_xbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), () -> MathUtil.applyDeadband(-m_xbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), () -> MathUtil.applyDeadband(-m_xbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND), () -> MathUtil.applyDeadband(-m_xbox.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND), false));
    String[] autoList = {"Do Nothing"};
    SmartDashboard.putStringArray("Auto List", autoList);
    initializeAutoChooser();
    System.out.print(I2C.Port.kMXP);
  }

  // update shuffleboard layout
  public void updateShuffleboard() {
  }

  public void initializeAutoChooser() {
    // with string chooser
    
    // with command chooser
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    main.add("Auto Routine", m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  // assign button functions
  private void configureButtonBindings() {
    Trigger xButton = new Trigger(m_xbox::getAButtonPressed);  
    xButton.onTrue(new AbsoluteDriveWithFocus(m_drivetrain, 
                    () -> MathUtil.applyDeadband(-m_xbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                    () -> MathUtil.applyDeadband(-m_xbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 
                    false, "cone"));
  }

  public Command getAutoInput() {
    return m_autoChooser.getSelected();
  }
}