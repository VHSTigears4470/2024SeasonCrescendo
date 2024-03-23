package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.OdometryLLConstants;
import frc.robot.Constants.OdometryLLConstants.ODOM_LL_PIPELINE;
import frc.robot.util.ListDebugEntry;

public class OdomLimelight extends SubsystemBase {

    // Shuffleboard
    private ShuffleboardTab shuffleDebugTab;
    private ListDebugEntry entry_pipelineName;

    public OdomLimelight() {
        // Default pipeline
        setPipeline(OdometryLLConstants.DEFAULT_PIPELINE);

        // Init shuffleboard
        initializeShuffleboard();
    }

    /**
     * Changes the pipeline.
     * 
     * @param pipeline is the pipeline to switch to
     */
    public void setPipeline(ODOM_LL_PIPELINE pipeline) {
        LimelightHelpers.setPipelineIndex(OdometryLLConstants.LIMELIGHT_NAME, pipeline.ordinal());
    }

    /**
     * Method to get the selected pipeline
     * 
     * @return Returns the pipeline
     */
    public ODOM_LL_PIPELINE getPipeline() {
        int ordinal = (int) LimelightHelpers.getCurrentPipelineIndex(OdometryLLConstants.LIMELIGHT_NAME);
        return ODOM_LL_PIPELINE.values()[ordinal];
    }

    // We suspect that red and blue need to be used because pose estimator specifies
    // "everything except april tags and auton are based on blue"
    // So using red will get us coordinates as if we were blue
    /**
     * Returns the Pose2D reported by the limelight, null if none
     * 
     * @return The Pose2D reported by the limelight, null if none
     */
    public Pose2d getPose2D() {
        // Checks if there is a target or not
        if (LimelightHelpers.getTV(OdometryLLConstants.LIMELIGHT_NAME)) {
            return LimelightHelpers.getBotPose2d_wpiBlue(OdometryLLConstants.LIMELIGHT_NAME);
        }
        return null;
    }

    /**
     * Returns the time the pose was captured in seconds
     * 
     * @return The time the pose was captured in seconds
     */
    public double getLatency() {
        return Timer.getFPGATimestamp()
                - (LimelightHelpers.getLatency_Pipeline(OdometryLLConstants.LIMELIGHT_NAME) / 1000.0)
                - (LimelightHelpers.getLatency_Capture(OdometryLLConstants.LIMELIGHT_NAME) / 1000.0);
    }

    /*** Inits Shuffleboard */
    private void initializeShuffleboard() {
        if (OdometryLLConstants.DEBUG) {
            shuffleDebugTab = Shuffleboard.getTab("Debug Tab");
            entry_pipelineName = new ListDebugEntry(shuffleDebugTab, "Odometry Limelight", "Current Pipeline",
                    "Default",
                    BuiltInWidgets.kTextView, false);
        }
    }

    /*** Updates Shuffleboard */
    private void updateShuffleboard() {
        if (OdometryLLConstants.DEBUG) {
            entry_pipelineName.set(getPipeline().toString());
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
