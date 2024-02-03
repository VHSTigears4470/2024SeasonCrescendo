package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    // Variables for intake pneumatics deployment and retractaction
    private final Compressor compressor;
    private final DoubleSolenoid intakePositionSolenoid;
    private final DoubleSolenoid notePusherSolenoid;

    
    // Variables for intake motors
    private final CANSparkMax backMotor;
    private final CANSparkMax frontMotor;
    private final RelativeEncoder backEncoder;
    private final RelativeEncoder frontEncoder;

    // Variables for sensors
    private final DigitalInput noteBreakbeam;

    // Shuffleboard
    private ShuffleboardTab shuffleDebugTab;
    private GenericEntry entry_compressorPressure;
    private GenericEntry entry_compressorSwitch;
    private GenericEntry entry_noteBreakBeam;

    public IntakeSubsystem() {
        // Pneumatics initialization
        compressor = new Compressor(IntakeConstants.PCM_MODULE_ID, IntakeConstants.MODULE_TYPE);
        intakePositionSolenoid = new DoubleSolenoid(IntakeConstants.PCM_MODULE_ID, IntakeConstants.MODULE_TYPE,
                IntakeConstants.INTAKE_REVERSE_CHANNEL_ID,
                IntakeConstants.INTAKE_FORWARD_CHANNEL_ID);
        intakePositionSolenoid.set(IntakeConstants.INTAKE_DEFAULT_POSITION);
        notePusherSolenoid = new DoubleSolenoid(IntakeConstants.PCM_MODULE_ID, IntakeConstants.MODULE_TYPE,
                IntakeConstants.NOTES_REVERSE_CHANNEL_ID,
                IntakeConstants.NOTES_FORWARD_CHANNEL_ID); //Update Later
        notePusherSolenoid.set(IntakeConstants.PISTON_DEFAULT_POSITION);

        // Motor initialization
        backMotor = new CANSparkMax(IntakeConstants.FRONT_MOTOR_ID, MotorType.kBrushless);
        backEncoder = backMotor.getEncoder();
        backMotor.setIdleMode(IdleMode.kBrake);
        backMotor.setInverted(false);
        backEncoder.setPosition(0);
        
        frontMotor = new CANSparkMax(IntakeConstants.FRONT_MOTOR_ID, MotorType.kBrushless);
        frontEncoder = frontMotor.getEncoder();
        frontMotor.setIdleMode(IdleMode.kBrake);
        frontMotor.setInverted(false);
        frontEncoder.setPosition(0);
        backMotor.follow(frontMotor);
        // Sensor initialization
        noteBreakbeam = new DigitalInput(IntakeConstants.NOTE_BREAKBEAM_RX_CHANNEL);

        initializeShuffleboard();
    }

    private void retractIntake() {
        intakePositionSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    private void retractPusher() {
        notePusherSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    private void extendIntake() {
        intakePositionSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    private void extendPusher() {
        notePusherSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setIntakeVoltage() {
        if (noteBreakbeam.get()) {
            setZeroVoltage();
        } else {
            frontMotor.setVoltage(IntakeConstants.INTAKE_VOLTAGE);
        }

    }

    public void setShootOutputVoltage() {
        frontMotor.setVoltage(IntakeConstants.SHOOT_OUTPUT_VOLTAGE);
    }

    public void setAmpOutputVoltage() {
        frontMotor.setVoltage(IntakeConstants.AMP_OUTPUT_VOLTAGE);
    }

    public void setNoteOutputVoltage() {
        frontMotor.setVoltage(IntakeConstants.NOTE_OUTPUT_VOLTAGE);
    }

    public void setZeroVoltage() {
        frontMotor.setVoltage(0);
    }

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
            entry_noteBreakBeam = shuffleDebugTab.getLayout("Intake", BuiltInLayouts.kList)
                    .add("Note Break Beam", false)
                    .withWidget(BuiltInWidgets.kBooleanBox)
                    .getEntry();
        }
    }

    private void updateShuffleboard() {
        if (IntakeConstants.DEBUG) {
            entry_compressorPressure.setDouble(compressor.getPressure());
            entry_compressorSwitch.setBoolean(compressor.getPressureSwitchValue());
            entry_noteBreakBeam.setBoolean(noteBreakbeam.get());
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
