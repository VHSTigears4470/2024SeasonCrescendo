package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

public final class Constants {
    public static final double K_SPEED = 0.5; //speed for turning
    public static final double K_WHEEL_DIAMETER_INCH = 6; //diameter of the wheels in inches
    public static final double K_TICKS_PER_FEET = 40.964489;

    /*----------
    AUTO
    -----------*/
    public static final double K_FORWARDS_FEET = 9* K_TICKS_PER_FEET; //17.666666
    public static final double K_BACKWARDS_FEET = 6 * K_TICKS_PER_FEET; //6.572500
    public static final float K_TURN_ERROR_RANGE = 2f;
    public static final double K_DEC_TO_PI = 0.01745;


    /*Drive train*/
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static final class Auton
    {
        public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

        public static final double MAX_SPEED        = 4;
        public static final double MAX_ACCELERATION = 2;
    }

    public static final class Drivebase
    {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants
    {
        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.01;
        public static final double LEFT_Y_DEADBAND = 0.01;
        public static final double RIGHT_X_DEADBAND = 0.01;
        public static final double RIGHT_Y_DEADBAND = 0.01;


    }
}
