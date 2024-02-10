package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.OdometryLLConstants;
import frc.robot.Constants.OdometryLLConstants.ODOM_LL_PIPELINE;

public class OdomLimelight extends SubsystemBase {

    // Shuffleboard
    private ShuffleboardTab shuffleDebugTab;
    private GenericEntry entry_pipelineName;

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
    public Pose2d getPose2D() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return LimelightHelpers.getBotPose2d_wpiRed(OdometryLLConstants.LIMELIGHT_NAME);
        }
        return LimelightHelpers.getBotPose2d_wpiBlue(OdometryLLConstants.LIMELIGHT_NAME);
    }

    /*** Inits Shuffleboard */
    private void initializeShuffleboard() {
        if (OdometryLLConstants.DEBUG) {
            shuffleDebugTab = Shuffleboard.getTab("Debug Tab");
            entry_pipelineName = shuffleDebugTab.getLayout("Odometry Limelight", BuiltInLayouts.kList)
                    .add("Current Pipeline", "Default")
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry();

        }
    }

    /*** Updates Shuffleboard */
    private void updateShuffleboard() {
        if (OdometryLLConstants.DEBUG) {
            entry_pipelineName.setString(getPipeline().toString());
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
