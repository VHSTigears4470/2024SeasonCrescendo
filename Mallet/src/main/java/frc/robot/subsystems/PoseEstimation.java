package frc.robot.subsystems;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
//import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.KRedAllianceWallRightSide;

import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

//import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix; 
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Constants.FieldConstants;
//import frc.robot.Constants.DrivetrainConstants;

public class PoseEstimation extends SubsystemBase {
    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d = new Field2d();
    private final PhotonVisionSubsystem rightEstimator = new PhotonRunnable(new PhotonCamera("rightCamera"),
        Constants.VisionConstants.ROBOT_TO_RIGHT_CAMERA);
    private final PhotonVisionSubsystem leftEstimator = new PhotonRunnable(new PhotonCamera("leftCamera"),
        Constants.VisionConstants.ROBOT_TO_LEFT_CAMERA);







}
