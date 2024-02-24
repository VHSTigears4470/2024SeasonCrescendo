package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DeadbandCommandXboxController extends CommandXboxController {
    private double leftXDeadband;
    private double leftYDeadband;
    private double rightXDeadband;
    private double rightYDeadband;

    /**
     * @param port     The port on the driver station that the controller is
     * @param deadband The deadband for all axes
     */
    public DeadbandCommandXboxController(int port, double deadband) {
        super(port);
        leftXDeadband = deadband;
        leftYDeadband = deadband;
        rightXDeadband = deadband;
        rightYDeadband = deadband;
    }

    /**
     * @param port           The port on the driver station that the controller is
     * @param leftXDeadband  The deadband for the left x-axis
     * @param leftYDeadband  The deadband for the left y-axis
     * @param rightXDeadband The deadband for the right x-axis
     * @param rightYDeadband The deadband for the right y-axis
     */
    public DeadbandCommandXboxController(int port, double leftXDeadband, double leftYDeadband, double rightXDeadband,
            double rightYDeadband) {
        super(port);
        this.leftXDeadband = leftXDeadband;
        this.leftYDeadband = leftYDeadband;
        this.rightXDeadband = rightXDeadband;
        this.rightYDeadband = rightYDeadband;
    }

    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(super.getLeftX(), leftXDeadband);
    }

    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(super.getLeftY(), leftYDeadband);
    }

    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(super.getRightX(), rightXDeadband);
    }

    @Override
    public double getRightY() {
        return MathUtil.applyDeadband(super.getRightY(), rightYDeadband);
    }
}