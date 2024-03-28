package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    // Variables for intake pneumatics deployment and retractaction
    private final Compressor compressor;
    private final DoubleSolenoid rightIntakePositionSolenoid;
    private final DoubleSolenoid leftIntakePositionSolenoid;
    private final DoubleSolenoid notePusherSolenoid;

    // Variables for intake motors
    private final CANSparkMax botMotor;
    private final CANSparkMax topMotor;
    private final RelativeEncoder botEncoder;
    private final RelativeEncoder topEncoder;

    // Variables for sensors
    // private final DigitalInput noteBreakbeam;

    // Shuffleboard
    private ShuffleboardTab shuffleDebugTab;
    private GenericEntry entry_compressorPressure;
    private GenericEntry entry_compressorSwitch;

    private Value intakeState;

    public IntakeSubsystem() {
        // Pneumatics initialization
        compressor = new Compressor(IntakeConstants.REVPH_MODULE_ID, IntakeConstants.MODULE_TYPE);
        rightIntakePositionSolenoid = new DoubleSolenoid(IntakeConstants.REVPH_MODULE_ID, IntakeConstants.MODULE_TYPE,
                IntakeConstants.RIGHT_INTAKE_REVERSE_CHANNEL_ID,
                IntakeConstants.RIGHT_INTAKE_FORWARD_CHANNEL_ID);
        rightIntakePositionSolenoid.set(IntakeConstants.INTAKE_DEFAULT_POSITION);
        leftIntakePositionSolenoid = new DoubleSolenoid(IntakeConstants.REVPH_MODULE_ID, IntakeConstants.MODULE_TYPE,
                IntakeConstants.LEFT_INTAKE_REVERSE_CHANNEL_ID,
                IntakeConstants.LEFT_INTAKE_FORWARD_CHANNEL_ID);
        leftIntakePositionSolenoid.set(IntakeConstants.INTAKE_DEFAULT_POSITION);
        intakeState = IntakeConstants.INTAKE_DEFAULT_POSITION;

        notePusherSolenoid = new DoubleSolenoid(IntakeConstants.REVPH_MODULE_ID, IntakeConstants.MODULE_TYPE,
                IntakeConstants.NOTES_REVERSE_CHANNEL_ID,
                IntakeConstants.NOTES_FORWARD_CHANNEL_ID); // Update Later
        notePusherSolenoid.set(IntakeConstants.PISTON_DEFAULT_POSITION);

        // Motor initialization
        botMotor = new CANSparkMax(IntakeConstants.BOT_MOTOR_ID, MotorType.kBrushless);
        botEncoder = botMotor.getEncoder();
        botMotor.setIdleMode(IdleMode.kBrake);
        botMotor.setInverted(false);
        botEncoder.setPosition(0);

        topMotor = new CANSparkMax(IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
        topEncoder = topMotor.getEncoder();
        topMotor.setIdleMode(IdleMode.kBrake);
        topMotor.setInverted(IntakeConstants.DIRECTION_INVERTED);
        topEncoder.setPosition(0);
        botMotor.follow(topMotor, IntakeConstants.FOLLOWER_INVERTED);

        // Sensor initialization
        // noteBreakbeam = new DigitalInput(IntakeConstants.NOTE_BREAKBEAM_DIO);

        initializeShuffleboard();
    }

    /*** Pivots the inake to the retracted ready for intaking */
    public void retractIntake() {
        leftIntakePositionSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightIntakePositionSolenoid.set(DoubleSolenoid.Value.kReverse);
        intakeState = DoubleSolenoid.Value.kReverse;
    }

    /*** Sets the feeder pistons ready for intaking */
    public void retractPusher() {
        notePusherSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /*** Pivots the intake to the ground, ready for intaking */
    public void extendIntake() {
        leftIntakePositionSolenoid.set(DoubleSolenoid.Value.kForward);
        rightIntakePositionSolenoid.set(DoubleSolenoid.Value.kForward);
        intakeState = DoubleSolenoid.Value.kForward;
    }

    /*** Feeds the note into the intake */
    public void extendPusher() {
        notePusherSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    /*** Set intake motors to the speed for intaking notes */
    public void setIntakeVoltage() {
        topMotor.setVoltage(-IntakeConstants.NOTE_INTAKE_VOLTAGE); // Negative for intaking
    }

    /***
     * Set intake motors to the speed for shooting the note to the speaker notes
     */
    public void setSpeakerOutputVoltage() {
        topMotor.setVoltage(IntakeConstants.SPEAKER_OUTPUT_VOLTAGE);
    }

    /*** Set intake motors to the speed for shooting the note to the amp */
    public void setAmpOutputVoltage() {
        topMotor.setVoltage(IntakeConstants.AMP_OUTPUT_VOLTAGE);
    }

    /*** Set intake motors to the speed for intaking notes */
    public void setNoteIntakeVoltage() {
        topMotor.setVoltage(-IntakeConstants.NOTE_INTAKE_VOLTAGE);
    }

    /*** Set intake motors to the speed for outputting notes at slow voltage */
    public void setSlowOutputVoltage() {
        topMotor.setVoltage(IntakeConstants.SLOW_OUTPUT_VOLTAGE);
    }

    /*** Stops motors */
    public void setZeroVoltage() {
        topMotor.setVoltage(0);
    }

    public Value getIntakeState() {
        return intakeState;
    }

    /*** Inits Shuffleboard */
    private void initializeShuffleboard() {
        if (IntakeConstants.DEBUG) {
            shuffleDebugTab = Shuffleboard.getTab("Debug Tab");
            entry_compressorPressure = shuffleDebugTab.getLayout("Intake", BuiltInLayouts.kList)
                    .add("Compressor Pressure", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry();
            entry_compressorSwitch = shuffleDebugTab.getLayout("Intake", BuiltInLayouts.kList)
                    .add("Compressor Switch", false)
                    .withWidget(BuiltInWidgets.kBooleanBox)
                    .getEntry();

        }
    }

    /*** Updates Shuffleboard */
    private void updateShuffleboard() {
        if (IntakeConstants.DEBUG) {
            entry_compressorPressure.setDouble(compressor.getPressure());
            entry_compressorSwitch.setBoolean(compressor.getPressureSwitchValue());
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateShuffleboard();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
