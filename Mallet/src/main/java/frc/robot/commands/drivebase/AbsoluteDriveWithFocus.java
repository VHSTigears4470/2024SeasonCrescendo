// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PathplannerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class AbsoluteDriveWithFocus extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final double Kp = -0.1; // Proportional control constant
    private final double minCommand = 0.05;
    private final double maxSpeed = 0.5; // Max speed of drivetrain (used for both positive and negative directions)
    private NetworkTableInstance inst;
    private NetworkTable table;
    private NetworkTableEntry tx;

    /**
     * Used to drive a swerve robot in full field-centric mode. vX and vY supply
     * translation inputs, where x is
     * torwards/away from alliance wall and y is left/right. headingHorzontal and
     * headingVertical are the Cartesian
     * coordinates from which the robot's angle will be derived— they will be
     * converted to a polar angle, which the robot
     * will rotate to.
     *
     * @param swerve The swerve drivebase subsystem.
     * @param vX     DoubleSupplier that supplies the x-translation joystick
     *               input. Should be in the range -1
     *               to 1 with deadband already accounted for. Positive X is
     *               away from the alliance wall.
     * @param vY     DoubleSupplier that supplies the y-translation joystick
     *               input. Should be in the range -1
     *               to 1 with deadband already accounted for. Positive Y is
     *               towards the left wall when
     *               looking through the driver station glass.
     */
    public AbsoluteDriveWithFocus(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
            String focusObject) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;

        // Init Network Tables
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("limelight");
        tx = table.getEntry("tx");

        // Sets the correct pipeline
        if (focusObject.toLowerCase().equals("pole"))
            table.getEntry("pipeline").setDouble(0); // pole pipeline
        else if (focusObject.toLowerCase().equals("cube"))
            table.getEntry("pipeline").setDouble(2); // cube pipeline
        else
            table.getEntry("pipeline").setDouble(3); // cone pipeline
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Grabs the heading error from the network table and uses it to create a
        // direction to go to
        double headingError = tx.getDouble(0.0);
        double steeringAdjust = Kp * headingError + Math.signum(headingError) * minCommand;
        // Formats steeringAdjust to be a number from -1 to 1
        steeringAdjust = Math.signum(steeringAdjust) * Math.min(Math.abs(steeringAdjust), maxSpeed);
        // Multiply it by a negative or positive depending on if it need to be inverted
        // it or not
        steeringAdjust *= 1;

        // Get the desired chassis speeds based on a 2 joystick module.

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                new Rotation2d(steeringAdjust * Math.PI));

        // Limit velocity to prevent tippy
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                PathplannerConstants.LOOP_TIME, PathplannerConstants.ROBOT_MASS, List.of(PathplannerConstants.CHASSIS),
                swerve.getSwerveDriveConfiguration());

        // Make the robot move
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
