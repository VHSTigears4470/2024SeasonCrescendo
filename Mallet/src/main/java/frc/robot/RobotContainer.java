package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.HashMap;

public class RobotContainer {
  // INIT SUBSYSTEMS
  private static final Drivetrain m_drivetrain = new Drivetrain();
  
  // INIT JOYSTICKS (NOTE: PLEASE RENAME TO LEFT/RIGHT)
  public static Joystick m_controller_arm = new Joystick(0);
  public static Joystick m_controller_drive = new Joystick(1);

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
    String[] autoList = {"Do Nothing"};
    SmartDashboard.putStringArray("Auto List", autoList);
    initializeAutoChooser();
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
    m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain, m_controller_drive));
    
    // Add joystick buttons to maps
    controllerButtons_drive.put("trigger", new JoystickButton(m_controller_drive, 1));
    controllerButtons_arm.put("trigger", new JoystickButton(m_controller_arm, 1));
    for (int i = 1; i <= 11; i++)
    {
      controllerButtons_arm.put(Integer.toString(i), new JoystickButton(m_controller_arm, i));
      controllerButtons_drive.put(Integer.toString(i), new JoystickButton(m_controller_drive, i));
    }

    //DRIVE CONTROLLER
    controllerButtons_drive.get("8").onTrue(new ResetEncoders(m_drivetrain));
  }

  public Command getAutoInput() {
    return m_autoChooser.getSelected();
  }

  public static Joystick getDriveController() {
    return m_controller_drive;
  }

  public static Joystick getArmController() {
    return m_controller_arm;
  }
}