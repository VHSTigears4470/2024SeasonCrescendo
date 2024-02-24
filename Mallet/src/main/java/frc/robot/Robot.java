package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  // runs only when the robot is first started
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "Crescendo2024"); // Set a metadata value

    if (isReal()) {
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.

    m_robotContainer = new RobotContainer();
    // addPeriodic(m_robotContainer::updateShuffleboard, .04, .005);
    disabledTimer = new Timer();
  }

  // called periodically regardless of mode
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // called when the robot is disabled
  @Override
  public void disabledInit() {
    disabledTimer.reset();
    disabledTimer.start();
  }

  // called continuously while robot is disabled
  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.SwerveConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  // called when auto is selected
  @Override
  public void autonomousInit() {
    m_robotContainer.getAutoInput().schedule();
  }

  // called periodically when robot is in auto
  @Override
  public void autonomousPeriodic() {
  }

  // called when teleop is selected
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.initializeDriveMode();
    m_robotContainer.setMotorBrake(true);
  }

  // called periodically when robot is in teleop
  @Override
  public void teleopPeriodic() {
  }

  // called when test is selected
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  // called periodically when robot is in test
  @Override
  public void testPeriodic() {
  }
}
