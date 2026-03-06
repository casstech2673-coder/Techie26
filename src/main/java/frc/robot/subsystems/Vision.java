package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

    // Arducam (PhotonVision) — global positioning
    private final PhotonCamera m_arducam;
    private final PhotonPoseEstimator m_photonPoseEstimator;

    // Limelight names (must match names set in each Limelight's Web UI)
    private static final String GLOBAL_LIMELIGHT = "limelight-3a"; // Fixed to robot, used for global pose
    private static final String TURRET_LIMELIGHT  = "limelight-4";  // Mounted on turret, used for target lock

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
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCam
        );
    }

    @Override
    public void periodic() {
        // limelight-3a and Arducam feed global pose into drivetrain odometry
        updatePoseFromLimelight(GLOBAL_LIMELIGHT);
        updatePoseFromArducam();
        // limelight-4 (turret) is NOT used for pose — only for target lock via getTurretTxDeg()
    }

    // =========================================================================
    // GLOBAL POSE ESTIMATION (limelight-3a + Arducam)
    // =========================================================================

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
        Optional<EstimatedRobotPose> poseResult = m_photonPoseEstimator.update(result);

        if (poseResult.isPresent()) {
            EstimatedRobotPose estPose = poseResult.get();
            m_swerve.addVisionMeasurement(estPose.estimatedPose.toPose2d(), estPose.timestampSeconds);
        }
    }

    // =========================================================================
    // TURRET TARGET LOCK (limelight-4, mounted on turret)
    // =========================================================================

    /**
     * Returns true if limelight-4 currently sees a valid target.
     */
    public boolean turretHasTarget() {
        return LimelightHelpers.getTV(TURRET_LIMELIGHT);
    }

    /**
     * Returns the horizontal offset (tx) from limelight-4 to the primary target, in degrees.
     * Positive = target is to the right of the crosshair.
     * Only meaningful when turretHasTarget() is true.
     */
    public double getTurretTxDeg() {
        return LimelightHelpers.getTX(TURRET_LIMELIGHT);
    }

    /**
     * Returns the vertical offset (ty) from limelight-4 to the primary target, in degrees.
     * Positive = target is above the crosshair.
     * Only meaningful when turretHasTarget() is true.
     */
    public double getTurretTyDeg() {
        return LimelightHelpers.getTY(TURRET_LIMELIGHT);
    }
}
