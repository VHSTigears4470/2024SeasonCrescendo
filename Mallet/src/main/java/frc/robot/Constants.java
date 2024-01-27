package frc.robot;

// Swerve Imports
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

// Intake Imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

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

    public static final class IntakeConstants {
        public static final boolean DEBUG = true;

        // Pneumatics
        public static final int PCM_MODULE_ID = 0;
        public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int FORWARD_CHANNEL_ID = 0;
        public static final int REVERSE_CHANNEL_ID = 1;
        public static final Value DEFAULT_POSITION = DoubleSolenoid.Value.kReverse;

        // Motors
        public static final int MOTOR_CAN = 0; // TODO: Update
        public static final double INTAKE_VOLTAGE = 2; // unsigned, polarity set in subsystem
        public static final double OUTPUT_VOLTAGE = 2; // unsigned, polarity set in subsystem

        // Sensors
        public static final int NOTE_BREAKBEAM_RX_CHANNEL = 0;
    }

    public static class OperatorConstants {
        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.05;
        public static final double LEFT_Y_DEADBAND = 0.05;
        public static final double RIGHT_X_DEADBAND = 0.01;
        public static final double RIGHT_Y_DEADBAND = 0.01;

    }
}
