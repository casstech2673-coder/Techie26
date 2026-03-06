package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class DriveConstants {
        // TODO: Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // TODO: Drivetrain CAN IDs
        public static final int kFrontLeftDrivingCanId = 11;
        public static final int kRearLeftDrivingCanId = 13;
        public static final int kFrontRightDrivingCanId = 15;
        public static final int kRearRightDrivingCanId = 17;

        public static final int kFrontLeftTurningCanId = 10;
        public static final int kRearLeftTurningCanId = 12;
        public static final int kFrontRightTurningCanId = 14;
        public static final int kRearRightTurningCanId = 16;

        public static final int kGyroCanId = 17;
        public static final boolean kGyroReversed = false;

        // TODO: Heading Lock PID Constants (You will need to tune the P value)
        public static final double kHeadingP = 5.0; 
        public static final double kHeadingI = 0.0;
        public static final double kHeadingD = 0.0;

        // --- PATHPLANNER AUTO PIDS ---
        // Translation PID (Corrects X/Y position errors during autonomous)
        // TODO: Tune this if your robot doesn't drive far enough or overshoots in Auto
        public static final double kAutoDriveP = 5.0; 
        public static final double kAutoDriveI = 0.0;
        public static final double kAutoDriveD = 0.0;

        // Rotation PID (Corrects heading errors during autonomous)
        // TODO: Tune this if your robot doesn't spin to the exact correct angle in Auto
        public static final double kAutoTurnP = 5.0; 
        public static final double kAutoTurnI = 0.0;
        public static final double kAutoTurnD = 0.0;
        
        // Max rotation speed and acceleration for the snap
        public static final double kMaxTurnRateRadiansPerSecond = Math.PI; // Half a circle per second
        public static final double kMaxTurnAccelerationRadiansPerSecondSquared = Math.PI * 2;

        // TODO: The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;
        public static final double kFreeSpeedRpm = 5676;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
    }

    public static final class ShooterConstants {
        public static final int kTurretId = 40;
        public static final int kHoodId = 41;
        public static final int kLeftFlywheelId = 42;
        public static final int kRightFlywheelId = 43;

        // --- TURRET RESTRAINTS (IN MOTOR ROTATIONS) ---
        // The exact number of Kraken motor rotations needed to spin the turret one full 360-degree circle.
        public static final double kTurretMotorRotationsPerTurretRevolution = 100.0; // TODO: DO NOT GUESS. You MUST calculate this via CAD gear ratios or measure it physically using the Spin Test on SmartDashboard.
        
        // TODO: Physical hard-stops
        public static final double kTurretMaxRotations = 41.6; 
        public static final double kTurretMinRotations = -41.6;

        // TODO: PID GAINS
        public static final double kFlywheelP = 0.5;
        public static final double kFlywheelI = 0.0;
        public static final double kFlywheelD = 0.0;
        public static final double kFlywheelV = 0.12; // Feedforward (Crucial for Velocity)

        public static final double kPositionP = 2.0;
        public static final double kPositionI = 0.0;
        public static final double kPositionD = 0.0;

        // --- TOLERANCES (ENCODER SPACE) ---
        public static final double kTurretToleranceRotations = 0.5;
        public static final double kHoodToleranceRotations = 0.2;
        public static final double kFlywheelToleranceRPS = 2.0; // Rotations Per Second
    }

    public static final class IntakeConstants {
        // TODO: CAN IDs
        public static final int kPivotMotorId = 20;
        public static final int kRollerMotorId = 21;

        // TODO: Angles (in Degrees) - YOU MUST MEASURE THESE ON THE REAL ROBOT!
        // The REV Through-Bore reads from 0 to 360 degrees.
        public static final double kStowedAngle = 95.0;
        public static final double kDeployedAngle = 10.0;

        // Roller Speed
        public static final double kIntakeRollerSpeed = 0.8; // TODO: 80% power
        public static final double kCreepSpeed = 0.05;       // 5% — slow/test mode

        // TODO: Pivot PID Gains (Needs tuning)
        public static final double kPivotP = 0.05;
        public static final double kPivotI = 0.0;
        public static final double kPivotD = 0.0;
    }

    public static final class HopperConstants {
        public static final int kVortexId = 30;
        
        // Speed to feed the game piece into the shooter flywheels.
        // TODO: You want this fast enough to not lose momentum, but not so fast that it skips on the ball.
        public static final double kFeedSpeed = 0.8; // 80% power
        public static final double kCreepSpeed = 0.05;       // 5% — slow/test mode
    }

    public static final class VisionConstants {

        // --- PHOTONVISION ---
        // TODO: This MUST exactly match the camera name you configured in the PhotonVision Web UI
        // running on the Luma.
        public static final String kArducamName = "Arducam_OV9281";

        // --- VISION REJECTION THRESHOLDS ---
        // If the robot is spinning faster than this (in degrees per second), we ignore AprilTag
        // pose updates because motion blur and rolling shutter will give us highly inaccurate data.
        // 360 degrees/sec means one full rotation per second. Tune this down if your odometry jumps while spinning.
        public static final double kMaxAngularVelocityForVision = 360.0;

        // --- LIMELIGHT-4 TURRET CAMERA GEOMETRY ---
        // TODO: Measure these from your robot CAD / physical robot
        // Height of the limelight-4 lens above the floor (meters)
        public static final double kTurretCameraHeightMeters = 0.60;
        // Fixed upward tilt of the camera from horizontal (degrees, positive = angled up)
        public static final double kTurretCameraMountAngleDeg = 20.0;
        // Height of the target center above the floor (meters) — e.g. hub opening
        // TODO: Confirm from game manual
        public static final double kTargetHeightMeters = 2.60;
    }

    public static final class SuperstructureConstants {
        // TODO: Field Coordinates for the Hub (double check these)
        // Hub Coordinates (Aiming at the center of the Hexagon)
        public static final double kBlueHubX = 8.27; // Center of field roughly
        public static final double kBlueHubY = 4.10; 
        
        public static final double kRedHubX = 8.27;
        public static final double kRedHubY = 4.10;

        // Alliance Wall Coordinates (For Passing/Passing)
        public static final double kBlueAllianceWallX = 0.0;
        public static final double kRedAllianceWallX = 16.54; // Roughly the field length
    }
}