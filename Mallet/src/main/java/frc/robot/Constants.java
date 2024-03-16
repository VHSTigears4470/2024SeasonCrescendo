package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
// Swerve Imports
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

// Intake Imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class Constants {
    public static final class RobotContainerConstants {
        public static final int XBOX_1_ID = 0;
        public static final double XBOX_1_DEADBAND = 0.2;
    }

    public static final class SwerveConstants {
        public static final boolean USING_SWERVE = true;
        public static final boolean DEBUG = false;

        public static final PIDFConfig X_AUTO_PID = new PIDFConfig(0.000000005, 0, 0);
        public static final PIDFConfig Y_AUTO_PID = new PIDFConfig(0.000000005, 0, 0);
        public static final PIDFConfig ANGLE_AUTO_PID = new PIDFConfig(0.01, 0, 0.04);

        public static final double ANGLE_KP = -0.1; // Proportional control constant
        public static final double ANGLE_MIN_TURN_SPEED = 0.05; // Min speed of drivetrain (used for both positive and negative directions)
        public static final double ANGLE_MAX_TURN_SPEED = 0.5; // Max speed of drivetrain (used for both positive and negative directions)
        public static final int ANGLE_INVERT_MULTIPLIER = 1; // Multiplier depending on if turning direction need to be inverted it or not
        public static final double OFFSET_THRESHOLD = 0.5; //TODO - change value

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
        public static final boolean IS_USING_INTAKE = false;
        public static final boolean DEBUG = true;

        // Pneumatics
        public static final int PCM_MODULE_ID = 0;
        public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int LEFT_INTAKE_FORWARD_CHANNEL_ID = 0;
        public static final int LEFT_INTAKE_REVERSE_CHANNEL_ID = 1;
        public static final int RIGHT_INTAKE_FORWARD_CHANNEL_ID = 0;
        public static final int RIGHT_INTAKE_REVERSE_CHANNEL_ID = 1;
        public static final int NOTES_FORWARD_CHANNEL_ID = 0;
        public static final int NOTES_REVERSE_CHANNEL_ID = 1;
        public static final Value INTAKE_DEFAULT_POSITION = DoubleSolenoid.Value.kReverse;
        public static final Value PISTON_DEFAULT_POSITION = DoubleSolenoid.Value.kReverse; // Need to change later for
                                                                                           // piston

        // Motors
        public static final int FRONT_MOTOR_ID = 0; // TODO: Update
        public static final int BACK_MOTOR_ID = 1;
        public static final double NOTE_INTAKE_VOLTAGE = 2; // unsigned, polarity set in subsystem
        public static final double SPEAKER_OUTPUT_VOLTAGE = 2; // unsigned, polarity set in subsystem
        public static final double AMP_OUTPUT_VOLTAGE = 2; // unsigned, polarity set in subsystem
        public static final double NOTE_OUTPUT_VOLTAGE = 2;// unsigned, polarity set in subsystem

        // Sensors
        public static final int NOTE_BREAKBEAM_RX_CHANNEL = 0;

        // Piston
        public static final int PISTON_ID = 0;

        public static enum INTAKE_POSITION_STATE {
            RETRACTED, UNRETRACTED
        };
    }

    public static class OperatorConstants {
        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.05;
        public static final double LEFT_Y_DEADBAND = 0.05;
        public static final double RIGHT_X_DEADBAND = 0.01;
        public static final double RIGHT_Y_DEADBAND = 0.01;

    }

    public static class ElevatorConstants {
        public static final boolean IS_USING_ELEVATOR = false;
        public static final boolean DEBUG = true;

        // Motors
        public static final int LEAD_MOTOR_ID = 5; // TODO: Update
        public static final int FOLLOW_MOTOR_ID = 6; // TODO: Update
        private static final double GEAR_RADIUS = 1;
        private static final double GEAR_CIRCUMFRENCE = 2 * Math.PI * GEAR_RADIUS;
        private static final double GEAR_RATIO = 60;
        public static final double CONVERSION_RATIO = GEAR_CIRCUMFRENCE / GEAR_RATIO;

        // Sensors
        public static final int BOTTOM_BREAKBEAM_CHANNEL_ID = 0; // TODO: Update
        public static final int TOP_BREAKBEAM_CHANNEL_ID = 0; // TODO: Update

        // PID
        public static final double PID_KP = 0.00000007015; // TODO: Tune
        public static final double PID_KI = 0; // TODO: Tune
        public static final double PID_KD = 0.0000000; // TODO: Tune
        public static final double PID_KIZ = 0.005; // TODO: Tune
        public static final double PID_KFF = 0.00031; // TODO: Tune
        // Must have both pos/neg for both directions
        public static final double PID_KMAX_OUTPUT = 5; // TODO: Tune
        public static final double PID_KMIN_OUTPUT = -5; // TODO: Tune

        // Smart Motion

        public static final int SM_ID = 0; // TODO: Update
        // Elevator maximum inches per second
        private static final double SM_MAX_INCHES_VEL = 6;
        // Elevator maximum revolutions per minute. Convert to revolutions and then
        // modify for gear ratio
        public static final double SM_MAX_RPM_VEL = SM_MAX_INCHES_VEL / GEAR_CIRCUMFRENCE
                * GEAR_RATIO * 60;

        // Elevator minimum inches per second
        private static final double SM_MIN_INCHES_OUTPUT_VEL = 0;
        // Elevator minimum revolutions per minute. Convert to revolutions and then
        // modify for gear ratio
        public static final double SM_MIN_RPM_OUTPUT_VEL = SM_MIN_INCHES_OUTPUT_VEL / GEAR_CIRCUMFRENCE
                * GEAR_RATIO * 60;

        // Elevator inches acceleration per second
        private static final double SM_INCHES_ACC = 10;
        // Elevator inches acceleration per minute. Convert to revolutions and then
        // modify for gear ratio
        public static final double SM_MAX_RPM_ACC = SM_INCHES_ACC / GEAR_CIRCUMFRENCE
                * GEAR_RATIO * 60;

        public static final double SM_ALLOWED_ERR = 0; // TODO: Update

        // Elevator States
        public static final double LOW_INIT_HEIGHT = 0;
        public static final double HIGH_INIT_HEIGHT = 30;

        public static enum ELEVATOR_STATE {
            UP, DOWN
        };

        // Tolerances
        public static final double HIGH_LOW_OFFSET = 0; // Distance in inches for PID to maintain from breakbeams
        public static final double POSITION_TOLERANCE = 1; // Distance in inches for PID to be considered "at the
                                                           // position"
    }

    public static class NoteLLConstants {
        public static final boolean IS_USING_NOTE_LIMELIGHT = false;
        public static final boolean DEBUG = true;

        public static final String LIMELIGHT_NAME = "limelight-note";
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

        public static final String LIMELIGHT_NAME = "limelight-april";
        public static final ODOM_LL_PIPELINE DEFAULT_PIPELINE = ODOM_LL_PIPELINE.APRIL;

        public static enum ODOM_LL_PIPELINE {
            APRIL;

            @Override
            public String toString() {
                return this.name();
            }
        }
    }

    public static class PhotonConstants {
        public static final boolean USING_VISION = true;

        public static final boolean USING_RIGHT_PHOTON = true;
        public static final boolean USING_LEFT_PHOTON = true;

        public static final String RIGHT_PHOTON_NAME = "right photon"; // TODO - Change both names
        public static final String LEFT_PHOTON_NAME = "left photon";

        public static final PoseStrategy PHOTON_CAMERA_STRAT = PoseStrategy.LOWEST_AMBIGUITY;

        // Right Camera w/ its real position
        public static final Transform3d ROBOT_TO_RIGHT_PHOTON = new Transform3d(
                new Translation3d(Units.inchesToMeters(-11.88), Units.inchesToMeters(-6.88),
                        Units.inchesToMeters(31.09)),
                new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(-45))); // TODO - Need to modify position of
                                                                                // Right

        // Left Camera w/ its real position
        public static final Transform3d ROBOT_TO_LEFT_PHOTON = new Transform3d(
                new Translation3d(Units.inchesToMeters(-11.88), Units.inchesToMeters(6.88),
                        Units.inchesToMeters(31.09)),
                new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(46))); // TODO - Need to modify position of Left

        // TODO - Review the var under to see if needs to be deleted
        // Min target ambiguity. Targets w/ higher ambiguity will be discarded
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
        public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
        public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
        public static final double NOISY_DISTANCE_METERS = 2.5;
        public static final double DISTANCE_WEIGHT = 7;
        public static final int TAG_PRESENCE_WEIGHT = 10; // TODO - Maybe modify these values(?)

        /**
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This
         * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
         * meters.
         */
        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(
                Nat.N3(), Nat.N1(),
                // if these numbers below are less than one, multiplying will do bad things
                1, // x
                1, // y
                1 * Math.PI // theta
        );
        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision
         * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
         * radians.
         */
        public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = MatBuilder.fill(
                Nat.N3(), Nat.N1(),
                // if these numbers are less than one, multiplying will do bad things
                .1, // x
                .1, // y
                .1);
    }

}
