package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteLLConstants;

public class NoteLimelight extends SubsystemBase {

    // Shuffleboard
    private ShuffleboardTab shuffleDebugTab;
    private GenericEntry entry_pipelineName;

    // Network Tables
    private NetworkTable table;
    private NetworkTableInstance instance;
    private NetworkTableEntry pipelineIndex;

    public NoteLimelight() {
        // Init all the pipelines
        // Default pipeline
        pipelineIndex.setDouble(NoteLLConstants.DEFAULT_PIPELINE_INDEX);

        // Init all the network tables
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("limelight");

        // Init shuffleboard
        initializeShuffleboard();
    }

    /**
     * Changes the pipeline index.
     * 
     * @param index is the pipeline index to switch to
     */
    public void setPipeline(int index) {
        pipelineIndex.setDouble(0);
    }

    /**
     * Method to get the selected pipeline's index
     * 
     * @return Returns the pipeline index
     */
    public int getPipelineIndex() {
        return (int) pipelineIndex.getInteger(NoteLLConstants.DEFAULT_PIPELINE_INDEX);
    }

    /*** Inits Shuffleboard */
    private void initializeShuffleboard() {
        if (NoteLLConstants.DEBUG) {
            shuffleDebugTab = Shuffleboard.getTab("Debug Tab");
            entry_pipelineName = shuffleDebugTab.getLayout("Limelight", BuiltInLayouts.kList)
                    .add("Limelight Name", getPipelineName())
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry();
        }
    }

    /*** Updates Shuffleboard */
    private void updateShuffleboard() {
        if (NoteLLConstants.DEBUG) {
            entry_pipelineName.setString(getPipelineName());
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
