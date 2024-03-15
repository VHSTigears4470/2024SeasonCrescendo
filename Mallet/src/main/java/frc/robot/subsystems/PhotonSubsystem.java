package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;

public class PhotonSubsystem extends SubsystemBase {
    // Field of April Tags
    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Physical Cameras
    private final PhotonCamera rightPhoton;
    private final PhotonCamera leftPhoton;

    // Pose Estimators
    private final PhotonPoseEstimator photonRightPoseEstimator;
    private final PhotonPoseEstimator photonLeftPoseEstimator;

    public PhotonSubsystem() {
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        if(PhotonConstants.USING_RIGHT_PHOTON) {
            rightPhoton = new PhotonCamera(PhotonConstants.RIGHT_PHOTON_NAME);
            photonRightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonConstants.PHOTON_CAMERA_STRAT, rightPhoton, PhotonConstants.ROBOT_TO_RIGHT_PHOTON);
        } else {
            rightPhoton = null;
            photonRightPoseEstimator = null;
        }

        if(PhotonConstants.USING_LEFT_PHOTON) {
            leftPhoton = new PhotonCamera(PhotonConstants.LEFT_PHOTON_NAME);
            photonLeftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonConstants.PHOTON_CAMERA_STRAT, leftPhoton, PhotonConstants.ROBOT_TO_LEFT_PHOTON);
        } else {
            leftPhoton = null;
            photonLeftPoseEstimator = null;
        }
    }
    /**
     * Reads from the left camera and then returns a pose
     * @param prevEstimatPose2d the latest estimated pose of the robot
     * @return a new estimated pose of the robot
     */
    public EstimatedRobotPose getEstimatedRobotPoseFromLeftPhoton(Pose2d prevEstimatPose2d) {
        // Checks if robot has a left photon camera
        if(PhotonConstants.USING_LEFT_PHOTON) {
            return null;
        }
        photonLeftPoseEstimator.setReferencePose(prevEstimatPose2d);
        Optional<EstimatedRobotPose> pose = photonLeftPoseEstimator.update();
        if(pose.isEmpty()) {
            return null;
        }
        return pose.get();
    }

    /**
     * Reads from the right camera and then returns a pose
     * @param prevEstimatPose2d the latest estimated pose of the robot
     * @return a new estimated pose of the robot
     */
    public EstimatedRobotPose getEstimatedRobotPoseFromRightPhoton(Pose2d prevEstimatPose2d) {
        // Checks if robot has a right photon camera
        if(PhotonConstants.USING_RIGHT_PHOTON) {
            return null;
        }
        photonRightPoseEstimator.setReferencePose(prevEstimatPose2d);
        Optional<EstimatedRobotPose> pose = photonLeftPoseEstimator.update();
        if(pose.isEmpty()) {
            return null;
        }
        return pose.get();
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}
