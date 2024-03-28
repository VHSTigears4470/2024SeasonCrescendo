package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.NoteLLConstants;
import frc.robot.Constants.NoteLLConstants.NOTE_LL_PIPELINE;

public class NoteLimelight extends SubsystemBase {

    // Shuffleboard
    private ShuffleboardTab shuffleDebugTab;
    private GenericEntry entry_pipelineName;
    private GenericEntry entry_tvEntry;
    private GenericEntry entry_txEntry;
    private GenericEntry entry_tyEntry;
    private GenericEntry entry_taEntry;

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

            entry_pipelineName = shuffleDebugTab.getLayout("Note Limelight", BuiltInLayouts.kList)
                    .add("Current Pipeline", "Default")
                    .withWidget(BuiltInWidgets.kTextView)
                    .withPosition(0, 5)
                    .withSize(4, 5)
                    .getEntry();
            entry_tvEntry = shuffleDebugTab.getLayout("Note Limelight", BuiltInLayouts.kList)
                    .add("Is Target", false)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withSize(4, 5)
                    .getEntry();
            entry_txEntry = shuffleDebugTab.getLayout("Note Limelight", BuiltInLayouts.kList)
                    .add("X Offset", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withSize(4, 5)
                    .getEntry();
            entry_tyEntry = shuffleDebugTab.getLayout("Note Limelight", BuiltInLayouts.kList)
                    .add("Y Offset", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withSize(4, 5)
                    .getEntry();
            entry_taEntry = shuffleDebugTab.getLayout("Note Limelight", BuiltInLayouts.kList)
                    .add("Area", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .withSize(4, 5)
                    .getEntry();
        }
    }

    /*** Updates Shuffleboard */
    private void updateShuffleboard() {
        if (NoteLLConstants.DEBUG) {
            entry_pipelineName.setString(getPipeline().toString());
            entry_tvEntry.setBoolean(hasTarget());
            entry_txEntry.setDouble(getXOffset());
            entry_tyEntry.setDouble(getYOffset());
            entry_taEntry.setDouble(getArea());
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
