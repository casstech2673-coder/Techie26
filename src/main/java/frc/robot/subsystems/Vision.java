package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.LimelightHelpers;

public class Vision extends SubsystemBase {

    private final SwerveDrive m_swerve;

    // Arducam (PhotonVision)
    private final PhotonCamera m_arducam;
    private final PhotonPoseEstimator m_photonPoseEstimator;

    // TODO: Limelight NetworkTable Names (Make sure to set these exact names in the Limelight Web UIs)
    private final String LEFT_LIMELIGHT = "limelight-left";
    private final String RIGHT_LIMELIGHT = "limelight-right";

    public Vision(SwerveDrive swerve) {
        this.m_swerve = swerve;
        
        this.m_arducam = new PhotonCamera(VisionConstants.kArducamName);

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        Transform3d robotToCam = new Transform3d(
                new Translation3d(0.2, 0.0, 0.5), 
                new Rotation3d(0, 0, 0)
        );

        m_photonPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, 
                robotToCam
        );
    }

    @Override
    public void periodic() {
        // Feed all three cameras into the Swerve Drive Odometry
        updatePoseFromLimelight(LEFT_LIMELIGHT);
        updatePoseFromLimelight(RIGHT_LIMELIGHT);
        updatePoseFromArducam();
    }

    // =========================================================================
    // MULTI-CAMERA ODOMETRY
    // =========================================================================

    /**
     * Grabs the MegaTag pose from a specific Limelight and sends it to the drivetrain.
     */
    private void updatePoseFromLimelight(String limelightName) {
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (poseEstimate != null && poseEstimate.tagCount > 0) {
            // Motion blur rejection
            if (Math.abs(m_swerve.getTurnRate()) > VisionConstants.kMaxAngularVelocityForVision) {
                return; 
            }
            m_swerve.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }
    }

    private void updatePoseFromArducam() {
        if (m_photonPoseEstimator == null) return;

        var result = m_arducam.getLatestResult();
        Optional<EstimatedRobotPose> poseResult = m_photonPoseEstimator.estimateCoprocMultiTagPose(result);

        if (poseResult.isEmpty()) {
            poseResult = m_photonPoseEstimator.estimateLowestAmbiguityPose(result);
        }

        if (poseResult.isPresent()) {
            EstimatedRobotPose estPose = poseResult.get();
            m_swerve.addVisionMeasurement(estPose.estimatedPose.toPose2d(), estPose.timestampSeconds);
        }
    }
}