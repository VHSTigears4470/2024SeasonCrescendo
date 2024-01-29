//DO NOT MODIFY THIS FILE!
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class ShooterSubsystem {
  private final CANSPARKMax m_topMotor = new CANSparkMax(topMotor, MotorType kBrushless);
  private final CANSPARKMax m_bottomMotor = new CANSparkMax(bottomMotor, MotorType kBrushless);

  public ShooterSubsystem() {
    RobotBase.startRobot(Robot::new);
    public double Stop(){
      return RobotBase.setVoltage(0);
    }
    public double Forward(double speed){
      return RobotBase.setVoltage(speed);
    }
    public double Backwards(double speed){
      return RobotBase.setVoltage(speed);
    }
  }
}
