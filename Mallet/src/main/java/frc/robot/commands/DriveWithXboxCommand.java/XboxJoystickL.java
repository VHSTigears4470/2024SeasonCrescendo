package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxJoyStickL extends CommandBase {

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain drivetrain;
    private final XboxController xboxController;

    public XboxJoyStickL(Drivetrain drivetrain, XboxController xboxController) {
        this.drivetrain = drivetrain;
        this.xboxController = xboxController;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("Starting Xbox drive command");
    }

    @Override
    public void execute() {
        double forwardSpeed = xboxController.getLeftY();
        double turningSpeed = xboxController.getLeftX();
        drivetrain.arcadeDrive(forwardSpeed, turningSpeed);
    }

    @Override
    public void end(boolean interruptted) {
        drivetrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}