package frc.robot;

import edu.wpWi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public final class Constants {
    public static final class SwerveConstants {
        public static final PIDFConfig X_AUTO_PID = new PIDFConfig(0.000000005, 0, 0);
        public static final PIDFConfig Y_AUTO_PID = new PIDFConfig(0.000000005, 0, 0);
        public static final PIDFConfig ANGLE_AUTO_PID = new PIDFConfig(0.01, 0, 0.04);

        // TODO CHANGE SCALAR in gettargetspeeds

        // SWERVE
        public static final TelemetryVerbosity TELEMETRY_VERBOSITY = TelemetryVerbosity.NONE;
        public static final double SPEED_SCALAR = 0.8;
        public static final double MAX_ACCELERATION = 2;
        public static final double MAX_SPEED_METERS = 2;

        public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
        public static final double WHEEL_LOCK_TIME = 10;
    }

    public static class OperatorConstants {
        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.05;
        public static final double LEFT_Y_DEADBAND = 0.05;
        public static final double RIGHT_X_DEADBAND = 0.01;
        public static final double RIGHT_Y_DEADBAND = 0.01;

    }
    
    public static class ElevatorConstants {
        // Motors
        public static final int LEAD_MOTOR_ID = 0;
        public static final int FOLLOW_MOTOR_ID = 0;
        public static final double GEAR_RATIO = 60;
        public static final double TICKS_PER_REV = 42.0 * GEAR_RATIO;

        // Limit Switches
        public static final int LIMIT_SWITCH_ID = 0;

        // PID
        public static final double PID_KP = 0;
        public static final double PID_KI = 0;
        public static final double PID_KD = 0;
        public static final double PID_KIZ = 0;
        public static final double PID_KFF = 0;
        public static final double PID_KMIN = 0;
        public static final double PID_KMAX_OUTPUT = 0;
        public static final double PID_KMIN_OUTPUT = 0;

        // Smart Motion
        public static final int SMART_MOTION_SLOT = 0;
        public static final double PID_MAX_VEL = 0;
        public static final double PID_MIN_VEL = 0;
        public static final double PID_MAX_ACCEL = 0;
        public static final double PID_ALLOWED_ERR = 0;

        // Elevator States
        public static enum ELEVATOR_STATES{UP, DOWN};
    }

}
