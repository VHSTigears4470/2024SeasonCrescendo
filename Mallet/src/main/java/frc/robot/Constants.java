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
    public static class OperatorConstants {
        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.05;
        public static final double LEFT_Y_DEADBAND = 0.05;
        public static final double RIGHT_X_DEADBAND = 0.01;
        public static final double RIGHT_Y_DEADBAND = 0.01;
    }

    public static final class RobotContainerConstants {
        public static final int XBOX_1_ID = 0;
        public static final int XBOX_2_ID = 1;
        public static final double XBOX_1_DEADBAND = 0.2;
    }

    public static final class SwerveConstants {
        public static final boolean USING_SWERVE = true;
        public static final boolean DEBUG = false;

        public static final PIDFConfig X_AUTO_PID = new PIDFConfig(0.000000005, 0, 0);
        public static final PIDFConfig Y_AUTO_PID = new PIDFConfig(0.000000005, 0, 0);
        public static final PIDFConfig ANGLE_AUTO_PID = new PIDFConfig(0.01, 0, 0.04);

        // TODO CHANGE SCALAR in gettargetspeeds

        // SWERVE
        public static final TelemetryVerbosity TELEMETRY_VERBOSITY = TelemetryVerbosity.MACHINE;
        public static final double SPEED_SCALAR = 0.8;
        public static final double MAX_ACCELERATION = 2;
        public static final double MAX_SPEED_METERS = 2;

        public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
        public static final double WHEEL_LOCK_TIME = 10;

        public static final double SWERVE_AUTO_VELOCITY = 1;
    }

    public static final class DifferentialConstants {
        public static final boolean USING_DIFFERENTIAL = false;
        public static final boolean DEBUG = true;

        // PWM Motors
        public static final int FL_MOTOR_PWM = 3;
        public static final int FR_MOTOR_PWM = 0;
        public static final int BL_MOTOR_PWM = 2;
        public static final int BR_MOTOR_PWM = 1;

        // Arcade Drive
        public static final double FWD_SPEED_SCALAR = 0.3;
        public static final double ROT_SPEED_SCALAR = 0.3;
    }

    public static final class IntakeConstants {
        public static final boolean IS_USING_INTAKE = true;
        public static final boolean DEBUG = true;

        // Pneumatics
        public static final int PCM_MODULE_ID = 0;
        public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int LEFT_INTAKE_FORWARD_CHANNEL_ID = 0;
        public static final int LEFT_INTAKE_REVERSE_CHANNEL_ID = 1;
        public static final int RIGHT_INTAKE_FORWARD_CHANNEL_ID = 2;
        public static final int RIGHT_INTAKE_REVERSE_CHANNEL_ID = 3;
        public static final int NOTES_FORWARD_CHANNEL_ID = 4;
        public static final int NOTES_REVERSE_CHANNEL_ID = 5;
        public static final Value INTAKE_DEFAULT_POSITION = DoubleSolenoid.Value.kReverse;
        public static final Value PISTON_DEFAULT_POSITION = DoubleSolenoid.Value.kReverse; // Need to change later for
                                                                                           // piston

        // Motors
        public static final int TOP_MOTOR_ID = 9; // Leader
        public static final int BOT_MOTOR_ID = 10; // Follower
        /** Inverts follower direction. */
        public static final boolean FOLLOWER_INVERTED = false; // False because we need them to spin in opposite
                                                               // directions to suck in
        /** Inverts both motors together. */
        public static final boolean DIRECTION_INVERTED = true; // Specifies sucking direction

        // Intake sucking and shooting speeds
        public static final double NOTE_INTAKE_VOLTAGE = 4; // unsigned, polarity set in subsystem
        public static final double SPEAKER_OUTPUT_VOLTAGE = 12; // unsigned, polarity set in subsystem
        public static final double AMP_OUTPUT_VOLTAGE = 3; // unsigned, polarity set in subsystem

        // Sensors
        public static final int NOTE_BREAKBEAM_RX_CHANNEL = 0;

        // Piston
        public static final int PISTON_ID = 0;

        public static enum INTAKE_POSITION_STATE {
            RETRACTED, UNRETRACTED
        };
    }

    public static class ElevatorConstants {
        public static final boolean IS_USING_ELEVATOR = false;
        public static final boolean DEBUG = true;

        // Motors
        public static final int LEFT_MOTOR_ID = 11; // Leader
        public static final int RIGHT_MOTOR_ID = 12; // Follower
        /** Inverts follower direction. */
        public static final boolean FOLLOWER_INVERTED = true; // True because they are not
                                                              // same orientation
        /** Inverts both motors together. */
        public static final boolean DIRECTION_INVERTED = true; // Specifies which
                                                               // direction is positive voltage

        // Height calculations
        private static final double GEAR_RADIUS = 1.981 / 2; // Sprocket WCP-0560, outermost radius 1.981
        private static final double GEAR_CIRCUMFRENCE = 2 * Math.PI * GEAR_RADIUS;
        private static final double GEAR_RATIO = 60;
        public static final double CONVERSION_RATIO = GEAR_CIRCUMFRENCE / GEAR_RATIO * 2; // TODO: CHECK IF NEED 2x IS
                                                                                          // CORRECT for 2 stage
                                                                                          // elevator

        // Sensors
        public static final int BOTTOM_BREAKBEAM_RX_CHANNEL = 0; // TODO: Update
        public static final int TOP_BREAKBEAM_RX_CHANNEL = 0; // TODO: Update

        // PID
        public static final double PID_KP = 0.00000007015; // TODO: Tune
        public static final double PID_KI = 0; // TODO: Tune
        public static final double PID_KD = 0.0000000; // TODO: Tune
        public static final double PID_KIZ = 0.005; // TODO: Tune
        public static final double PID_KFF = 0.00031; // TODO: Tune
        /** Max positive voltage (must be positive) */
        public static final double PID_KMAX_OUTPUT = 3; // TODO: Tune
        /** Max negative voltage (must be negative) */
        public static final double PID_KMIN_OUTPUT = -3; // TODO: Tune

        // Smart Motion
        public static final int SM_ID = 0; // TODO: Update
        private static final double SM_MAX_INCHES_VEL = 6; // Elevator maximum inches per second
        public static final double SM_MAX_RPM_VEL = SM_MAX_INCHES_VEL / GEAR_CIRCUMFRENCE
                * GEAR_RATIO * 60; // Elevator maximum revolutions per minute -> revolutions -> gear ratio

        private static final double SM_MIN_INCHES_VEL = 0; // Elevator minimum inches per second
        public static final double SM_MIN_RPM_VEL = SM_MIN_INCHES_VEL / GEAR_CIRCUMFRENCE
                * GEAR_RATIO * 60; // Elevator minimum revolutions per minute -> revolutions -> gear ratio

        private static final double SM_INCHES_ACC = 10; // Elevator inches acceleration per second
        public static final double SM_MAX_RPM_ACC = SM_INCHES_ACC / GEAR_CIRCUMFRENCE
                * GEAR_RATIO * 60; // Elevator inches acceleration per minute -> revolutions -> gear ratio

        public static final double SM_ALLOWED_ERR = 0; // TODO: Update

        // Elevator States
        public static final double LOW_INIT_HEIGHT = 0;
        public static final double HIGH_INIT_HEIGHT = 30;// 14, but actually 28 because 2 stage
        public static final double CLIMB_HEIGHT = 15; // TODO: Update

        public static enum ELEVATOR_STATE {
            UP, DOWN, CLIMB
        };

        // Tolerances
        public static final double HIGH_LOW_OFFSET = 0; // Distance in inches for PID to maintain from breakbeams
        public static final double POSITION_TOLERANCE = 1; // Distance in inches for PID to be considered "at the
                                                           // position"
    }

    public static class NoteLLConstants {
        public static final boolean IS_USING_NOTE_LIMELIGHT = false;
        public static final boolean DEBUG = true;

        public static final String LIMELIGHT_NAME = "";
        public static final NOTE_LL_PIPELINE DEFAULT_PIPELINE = NOTE_LL_PIPELINE.NOTES;

        // Use NOTES.ordinal to get its index, or in other words list the names in the
        // same order as pipelines in the limelight
        public static enum NOTE_LL_PIPELINE {
            NOTES;

            @Override
            public String toString() {
                return this.name();
            }
        }
    }

    public static class OdometryLLConstants {
        public static final boolean IS_USING_ODOM_LIMELIGHT = false;
        public static final boolean DEBUG = true;

        public static final String LIMELIGHT_NAME = "limelight-odometry";
        public static final ODOM_LL_PIPELINE DEFAULT_PIPELINE = ODOM_LL_PIPELINE.APRIL;

        public static enum ODOM_LL_PIPELINE {
            APRIL;

            @Override
            public String toString() {
                return this.name();
            }
        }
    }

}
