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
    public static final class OperatorConstants {
        public static final boolean USING_XBOX_1 = true;
        public static final boolean USING_XBOX_2 = false;

        public static final int XBOX_1_ID = 0;
        public static final double XBOX_1_DEADBAND = 0.2;

        public static final int XBOX_2_ID = 1;
        public static final double XBOX_2_DEADBAND = 0.2;
    }

    public static final class AutoConstants {
        public static final int numOfDirections = 8;

        // String Names for starting Position
        public static final String AMP_SIDE_START = "Amp Start";
        public static final String MIDDLE_SIDE_START = "Middle Start";
        public static final String FEEDER_SIDE_START = "Feeder Start";

        // String Names for Cycles
        public static final String AMP_WING_NOTE_ENDING = "Amp Wing";
        public static final String MIDDLE_WING_NOTE_ENDING = "Middle Wing";
        public static final String FEEDER_WING_NOTE_ENDING = "Feeder Wing";

        // String Names for Center Note Paths
        public static final String AMP_CENTER_NOTE_ENDING = "Center Left";
        public static final String AMP_MIDDLE_CENTER_NOTE_ENDING = "Center Middle Left";
        public static final String MIDDLE_CENTER_NOTE_ENDING = "Center Middle";
        public static final String FEEDER_MIDDLE_CENTER_NOTE_ENDING = "Center Right Middle";
        public static final String FEEDER_CENTER_NOTE_ENDING = "Center Right";

        // String Names for Named Commands
        public static final String CLIMB_POSITION = "Climb Position";
        public static final String DEFAULT_POSITION = "Default Position";
        public static final String INTAKE_POSITION = "Intake Position";
        public static final String INTAKE_NOTE = "Intake Note";
        public static final String SHOOT_AMP = "Shoot Amp";
        public static final String SHOOT_SPEAKER = "Shoot Speaker";
        public static final String DRIVE_TIL_HAVE_NOTE = "Drive Till Have Note";
    }

    public static final class CycleTimes {
        // All times are in seconds
        public static final double INTAKE_MOTORS_WARM_UP_CYCLE_TIME = 0.5;
        public static final double INTAKE_MOTORS_PUSHER_CYCLE_TIME = 0.7;
        public static final double CHANGE_POSITIONS_CYCLE_TIME = 0.5;
    }

    public static final class SwerveConstants {
        public static final boolean USING_SWERVE = true;
        public static final boolean DEBUG = true;

        public static final PIDFConfig X_AUTO_PID = new PIDFConfig(0.000000005, 0, 0);
        public static final PIDFConfig Y_AUTO_PID = new PIDFConfig(0.000000005, 0, 0);
        public static final PIDFConfig ANGLE_AUTO_PID = new PIDFConfig(0.01, 0, 0.04);

        public static final double ANGLE_KP = -0.1; // Proportional control constant
        public static final double ANGLE_MIN_TURN_SPEED = 0.05; // Min speed of drivetrain (used for both positive and
                                                                // negative directions)
        public static final double ANGLE_MAX_TURN_SPEED = 0.5; // Max speed of drivetrain (used for both positive and
                                                               // negative directions)
        public static final int ANGLE_INVERT_MULTIPLIER = 1; // Multiplier depending on if turning direction need to be
                                                             // inverted it or not
        public static final double OFFSET_THRESHOLD = 0.5; // TODO - change value

        // TODO CHANGE SCALAR in gettargetspeeds

        // SWERVE
        public static final TelemetryVerbosity TELEMETRY_VERBOSITY = TelemetryVerbosity.NONE;
        public static final double SPEED_SCALAR = 0.8;
        public static final double MAX_ACCELERATION = 2;
        public static final double MAX_SPEED_METERS = 2;

        public static final double ROBOT_MASS = Units.lbsToKilograms(128); //
        public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(0)), ROBOT_MASS);
        public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
        public static final double WHEEL_LOCK_TIME = 10;

        public static final double SWERVE_AUTO_VELOCITY = 1;

        // TOLERANCES
        public static final double HEADING_TOLERANCE = 10;
    }

    public static final class DifferentialConstants {
        public static final boolean USING_DIFFERENTIAL = false;
        public static final boolean DEBUG = true;

        // PWM MOTORS
        public static final int FL_MOTOR_PWM = 5;
        public static final int FR_MOTOR_PWM = 0;
        public static final int BL_MOTOR_PWM = 2;
        public static final int BR_MOTOR_PWM = 1;

        // ARCADE DRIVE
        public static final double FWD_SPEED_SCALAR = 0.3;
        public static final double ROT_SPEED_SCALAR = 0.3;
    }

    public static final class IntakeConstants {
        public static final boolean IS_USING_INTAKE = true;
        public static final boolean DEBUG = true;

        // PNEUMATICS
        public static final int REVPH_MODULE_ID = 1;
        public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.REVPH;
        public static final int LEFT_INTAKE_FORWARD_CHANNEL_ID = 8;
        public static final int LEFT_INTAKE_REVERSE_CHANNEL_ID = 13;
        public static final int RIGHT_INTAKE_FORWARD_CHANNEL_ID = 7;
        public static final int RIGHT_INTAKE_REVERSE_CHANNEL_ID = 2;
        public static final int NOTES_FORWARD_CHANNEL_ID = 11;
        public static final int NOTES_REVERSE_CHANNEL_ID = 10;
        public static final Value INTAKE_DEFAULT_POSITION = DoubleSolenoid.Value.kReverse;
        public static final Value PISTON_DEFAULT_POSITION = DoubleSolenoid.Value.kReverse;

        public static final double SECONDS_TILL_UP = 3;
        public static final double SECONDS_TILL_DOWN = 1.5;

        // MOTORS
        public static final int TOP_MOTOR_ID = 9; // Leader
        public static final int BOT_MOTOR_ID = 10; // Follower
        /** Inverts follower direction. */
        public static final boolean FOLLOWER_INVERTED = false; // False because we need them to spin in opposite
                                                               // directions to suck in
        /** Inverts both motors together. */
        public static final boolean DIRECTION_INVERTED = false; // Specifies sucking direction

        // Intake sucking and shooting speeds
        public static final double NOTE_INTAKE_VOLTAGE = 4; // unsigned, polarity set in subsystem
        public static final double SPEAKER_OUTPUT_VOLTAGE = 12; // unsigned, polarity set in subsystem
        public static final double AMP_OUTPUT_VOLTAGE = 3; // unsigned, polarity set in subsystem
        public static final double SLOW_OUTPUT_VOLTAGE = 2; // unsigned, polarity set in subsystem

        // SENSORS
        public static final int NOTE_BREAKBEAM_DIO = 3; // TODO: Set DIO
    }

    public static class ElevatorConstants {
        public static final boolean IS_USING_ELEVATOR = true;
        public static final boolean DEBUG = true;

        // MOTORS
        public static final int LEFT_MOTOR_ID = 11; // Leader
        public static final int RIGHT_MOTOR_ID = 12; // Follower
        /** Inverts follower direction. */
        public static final boolean FOLLOWER_INVERTED = true; // True because they are not
                                                              // same orientation
        /** Inverts both motors together. */
        public static final boolean DIRECTION_INVERTED = true; // Specifies which
                                                               // direction is positive voltage

        // HEIGHT CALCULATIONS
        private static final double GEAR_RADIUS = 2.15 / 2; // Sprocket WCP-0560, outermost radius 1.981, 1.756 inner
        private static final double GEAR_CIRCUMFRENCE = 2 * Math.PI * GEAR_RADIUS;
        private static final double GEAR_RATIO = 60;
        public static final double CONVERSION_RATIO = GEAR_CIRCUMFRENCE / GEAR_RATIO * 2; // * 2 for 2 stages

        // SENSORS
        public static final int BOTTOM_BREAKBEAM_DIO = 1; // TODO: Update
        public static final int TOP_BREAKBEAM_DIO = 4; // TODO: Update

        // PID
        public static final double PID_KP = 0.0000010015; // TODO: Tune
        public static final double PID_KI = 0; // TODO: Tune
        public static final double PID_KD = 0.0000000; // TODO: Tune
        public static final double PID_KIZ = 0.005; // TODO: Tune
        public static final double PID_KFF = 0.00071; // TODO: Tune
        /** Max positive voltage (must be positive) */
        public static final double PID_KMAX_OUTPUT = 3.5; // TODO: Tune
        /** Max negative voltage (must be negative) */
        public static final double PID_KMIN_OUTPUT = -1; // TODO: Tune

        // SMART MOTION
        public static final int SM_ID = 0;
        private static final double SM_MAX_INCHES_VEL = 4; // Elevator maximum inches per second
        public static final double SM_MAX_RPM_VEL = SM_MAX_INCHES_VEL / GEAR_CIRCUMFRENCE
                * GEAR_RATIO * 60; // Elevator maximum revolutions per minute -> revolutions -> gear ratio

        private static final double SM_MAX_INCHES_VEL_DOWN = 4; // Elevator maximum inches per second
        public static final double SM_MAX_RPM_VEL_DOWN = SM_MAX_INCHES_VEL_DOWN / GEAR_CIRCUMFRENCE
                * GEAR_RATIO * 60; // Elevator maximum revolutions per minute -> revolutions -> gear ratio

        private static final double SM_MIN_INCHES_VEL = 0; // Elevator minimum inches per second
        public static final double SM_MIN_RPM_VEL = SM_MIN_INCHES_VEL / GEAR_CIRCUMFRENCE
                * GEAR_RATIO * 60; // Elevator minimum revolutions per minute -> revolutions -> gear ratio

        private static final double SM_INCHES_ACC = 10; // Elevator inches acceleration per second
        public static final double SM_MAX_RPM_ACC = SM_INCHES_ACC / GEAR_CIRCUMFRENCE
                * GEAR_RATIO * 60; // Elevator inches acceleration per minute -> revolutions -> gear ratio

        public static final double SM_ALLOWED_ERR = 0; //

        // ELEVATOR STATES
        // TODO: Find correct heights
        public static final double LOW_INIT_HEIGHT = 0;
        public static final double HIGH_INIT_HEIGHT = 33.5; // amp height / .5 inch below hall sensor
        public static final double CLIMB_HEIGHT = 20;

        public static enum ELEVATOR_STATE {
            UP, DOWN, CLIMB
        };

        // TOLERANCES
        public static final double HIGH_LOW_OFFSET = 0.5; // Distance in inches for PID to maintain from breakbeams
        public static final double POSITION_TOLERANCE = 1; // Distance in inches for PID to be considered "at the
                                                           // position"
    }

    public static class NoteLLConstants {
        public static final boolean IS_USING_NOTE_LIMELIGHT = false;
        public static final boolean DEBUG = true;

        public static final double DISTANCE_TO_CENTER = 1; // TODO: UPDATE *INCHES

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
        public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
        public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(315.5);

        public static final boolean USING_VISION = true;

        public static final boolean USING_REAR_PHOTON = true;
        public static final boolean USING_ANGLED_PHOTON = false;

        public static final String REAR_PHOTON_NAME = "Arducam_Back_Camera"; // TODO - Change both names
        public static final String ANGLED_PHOTON_NAME = "leftCamera";

        public static final PoseStrategy PHOTON_CAMERA_STRAT = PoseStrategy.LOWEST_AMBIGUITY;

        // Rear Camera w/ its real position offset from center of robot and from ground
        public static final Transform3d ROBOT_TO_REAR_PHOTON = new Transform3d(
                new Translation3d(Units.inchesToMeters(-11.75), Units.inchesToMeters(2.25),
                        Units.inchesToMeters(19.3772)),
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(180)));

        // Right Camera w/ its real position offset from center of robot and from ground
        public static final Transform3d ROBOT_TO_ANGLED_PHOTON = new Transform3d(
                new Translation3d(Units.inchesToMeters(0.05), Units.inchesToMeters(-9.6036),
                        Units.inchesToMeters(6.6108)),
                new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(-90)));

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
