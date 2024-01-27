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
    private final DoubleSolenoid doubleSolenoid;

    // Variables for intake motors
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

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
        doubleSolenoid = new DoubleSolenoid(IntakeConstants.PCM_MODULE_ID, IntakeConstants.MODULE_TYPE,
                IntakeConstants.REVERSE_CHANNEL_ID,
                IntakeConstants.FORWARD_CHANNEL_ID);
        doubleSolenoid.set(IntakeConstants.DEFAULT_POSITION);

        // Motor initialization
        motor = new CANSparkMax(IntakeConstants.MOTOR_CAN, MotorType.kBrushless);
        encoder = motor.getEncoder();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(false);
        encoder.setPosition(0);

        // Sensor initialization
        noteBreakbeam = new DigitalInput(IntakeConstants.NOTE_BREAKBEAM_RX_CHANNEL);

        initializeShuffleboard();
    }

    public void setIntakeVoltage() {
        if (noteBreakbeam.get()) {
            setZeroVoltage();

        } else {
            motor.setVoltage(IntakeConstants.INTAKE_VOLTAGE);
        }

    }

    public void setOutputVoltage() {
        motor.setVoltage(IntakeConstants.OUTPUT_VOLTAGE);
    }

    public void setZeroVoltage() {
        motor.setVoltage(0);
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
