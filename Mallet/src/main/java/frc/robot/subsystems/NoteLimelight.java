package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.NoteLLConstants;
import frc.robot.Constants.NoteLLConstants.NOTE_LL_PIPELINE;
import frc.robot.util.ListDebugEntryBool;
import frc.robot.util.ListDebugEntryDouble;
import frc.robot.util.ListDebugEntryString;

public class NoteLimelight extends SubsystemBase {

    // Shuffleboard
    private ShuffleboardTab shuffleDebugTab;
    private ListDebugEntryString entry_pipelineName;
    private ListDebugEntryBool entry_tvEntry;
    private ListDebugEntryDouble entry_txEntry;
    private ListDebugEntryDouble entry_tyEntry;
    private ListDebugEntryDouble entry_taEntry;

    public NoteLimelight() {
        // Default pipeline
        setPipeline(NoteLLConstants.DEFAULT_PIPELINE);

        // Init shuffleboard
        initializeShuffleboard();
    }

    /**
     * Changes the pipeline.
     * 
     * @param pipeline is the pipeline to switch to
     */
    public void setPipeline(NOTE_LL_PIPELINE pipeline) {
        LimelightHelpers.setPipelineIndex(NoteLLConstants.LIMELIGHT_NAME, pipeline.ordinal());
    }

    /**
     * Method to get the selected pipeline
     * 
     * @return Returns the pipeline
     */
    public NOTE_LL_PIPELINE getPipeline() {
        int ordinal = (int) LimelightHelpers.getCurrentPipelineIndex(NoteLLConstants.LIMELIGHT_NAME);
        return NOTE_LL_PIPELINE.values()[ordinal];
    }

    /**
     * @return Whether or not the limelight has found a target
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(NoteLLConstants.LIMELIGHT_NAME);
    }

    /**
     * @return The horizontal offset from the target (-27 to 27 degrees)
     */
    public double getXOffset() {
        return LimelightHelpers.getTX(NoteLLConstants.LIMELIGHT_NAME);
    }

    /**
     * @return The vertical offset from the target (-20.5 to 20.5 degrees)
     */
    public double getYOffset() {
        return LimelightHelpers.getTY(NoteLLConstants.LIMELIGHT_NAME);
    }

    /**
     * @return The area of the target (0 to 100%)
     */
    public double getArea() {
        return LimelightHelpers.getTA(NoteLLConstants.LIMELIGHT_NAME);
    }

    /*** Inits Shuffleboard */
    private void initializeShuffleboard() {
        if (NoteLLConstants.DEBUG) {
            shuffleDebugTab = Shuffleboard.getTab("Debug Tab");
            // convert to ListDebugEntry

            entry_pipelineName = new ListDebugEntryString(shuffleDebugTab, "Note Limelight", "Current Pipeline",
                    "Default",
                    BuiltInWidgets.kTextView, false);
            entry_tvEntry = new ListDebugEntryBool(shuffleDebugTab, "Note Limelight", "Is Target", false,
                    BuiltInWidgets.kBooleanBox, true);
            entry_txEntry = new ListDebugEntryDouble(shuffleDebugTab, "Note Limelight", "X Offset", 0.0,
                    BuiltInWidgets.kTextView, true);
            entry_tyEntry = new ListDebugEntryDouble(shuffleDebugTab, "Note Limelight", "Y Offset", 0.0,
                    BuiltInWidgets.kTextView, true);
            entry_taEntry = new ListDebugEntryDouble(shuffleDebugTab, "Note Limelight", "Area", 0.0,
                    BuiltInWidgets.kTextView, true);
        }
    }

    /*** Updates Shuffleboard */
    private void updateShuffleboard() {
        if (NoteLLConstants.DEBUG) {
            entry_pipelineName.set(getPipeline().toString());
            entry_tvEntry.set(hasTarget());
            entry_txEntry.set(getXOffset());
            entry_tyEntry.set(getYOffset());
            entry_taEntry.set(getArea());
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
