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

    // Arducam (PhotonVision) — secondary global pose source
    private final PhotonCamera m_arducam;
    private final PhotonPoseEstimator m_photonPoseEstimator;

    // Limelight 4 — primary global pose source via MegaTag2.
    //
    // Physical position relative to robot center (WPILib frame: +X forward, +Y left, +Z up).
    // *** Configure these values in the Limelight web UI under "Robot Space" ***
    //
    //   Forward (X): -(29 - 0.5) / 2  in = -14.25 in  (-0.362 m)  [14.25 in behind center]
    //   Left    (Y): +(25/2 - 10)     in =  +2.50 in  (+0.064 m)  [10 in right of back-left corner]
    //   Up      (Z):  13.375          in              ( 0.340 m)  [13 3/8 in above floor]
    //   Yaw       :  180°  (camera faces rearward)
    //
    // limelight-4 is fixed to the robot chassis (not turret-mounted).
    private static final String GLOBAL_LIMELIGHT = "limelight-4";

    public Vision(SwerveDrive swerve) {
        this.m_swerve = swerve;

        this.m_arducam = new PhotonCamera(VisionConstants.kArducamName);

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // TODO: Update this transform to match the Arducam's actual mount position
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
        // Both sources feed the SwerveDrivePoseEstimator for fused odometry.
        updatePoseFromLimelight(GLOBAL_LIMELIGHT);
        updatePoseFromArducam();
    }

    // =========================================================================
    // GLOBAL POSE ESTIMATION
    // =========================================================================

    /**
     * MegaTag2 workflow:
     *  1. Push the robot's gyro heading to the Limelight each loop.
     *     Limelight uses this heading (instead of its internal IMU) to lock
     *     the rotational component of the pose — the result is a 2D (X, Y)
     *     fix that is much more stable than MegaTag1, even with only one tag.
     *  2. Grab the resulting pose estimate and inject it into odometry.
     */
    private void updatePoseFromLimelight(String limelightName) {
        // MegaTag2 requires the raw gyro heading (not odometry-fused pose) so
        // the Limelight can lock rotation and solve only X/Y. Also supply the
        // live yaw rate so MegaTag2 can compensate for camera-latency during rotation.
        double headingDeg = m_swerve.getHeading();
        double yawRateDegPerSec = m_swerve.getTurnRate();
        LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, yawRateDegPerSec, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate poseEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (poseEstimate != null && poseEstimate.tagCount > 0) {
            // Reject updates when spinning fast: motion blur and rolling shutter
            // make AprilTag detections unreliable above this angular rate.
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
}
