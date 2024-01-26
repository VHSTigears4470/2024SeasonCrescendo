package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  // runs only when the robot is first started
  @Override
  public void robotInit() {
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
    if (disabledTimer.hasElapsed(Constants.PathplannerConstants.WHEEL_LOCK_TIME)) {
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
    m_robotContainer.setDriveMode();
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
