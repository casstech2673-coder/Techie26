package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
    
    private final SparkMax m_vortex; // NEO on SparkMax
    
    // --- Timing variables for the "Run-Up" sequence ---
    private final Timer m_feedTimer = new Timer();
    private boolean m_isFeedingSequence = false;
    private static final double BACKSPIN_TIME_SECONDS = 0.5; //TODO: tune this on the real robot

    public Hopper() {
        m_vortex = new SparkMax(HopperConstants.kVortexId, MotorType.kBrushless);

        SparkMaxConfig hopperConfig = new SparkMaxConfig();
        hopperConfig.inverted(false); 
        hopperConfig.smartCurrentLimit(50); 
        hopperConfig.idleMode(IdleMode.kBrake);

        m_vortex.configure(hopperConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {}

    public double getVortexCurrent() { return m_vortex.getOutputCurrent(); }
    public boolean isFeeding() { return m_isFeedingSequence; }

    // ==========================================================
    // HOPPER COMMAND METHODS
    // ==========================================================

    /**
     * Called continuously by the Superstructure when the shooter is locked.
     * Automatically handles backing the ball off before firing.
     */
    public void startFeedSequence() {
        // If we just locked on, start the timer
        if (!m_isFeedingSequence) {
            m_isFeedingSequence = true;
            m_feedTimer.restart(); 
        }

        // Check how much time has passed since we locked on
        if (m_feedTimer.get() < BACKSPIN_TIME_SECONDS) {
            // Phase 1: Back the ball up
            m_vortex.set(-HopperConstants.kFeedSpeed);
        } else {
            // Phase 2: Slam it into the shooter
            m_vortex.set(HopperConstants.kFeedSpeed);
        }
    }

    /**
     * Called by the Superstructure when the shooter loses its lock.
     * Resets the sequence so it backs up again next time we lock on.
     */
    public void stop() {
        m_isFeedingSequence = false;
        m_feedTimer.stop();
        m_vortex.set(0.0);
    }

    public void reverse() {
        // Manual override for unjamming
        m_isFeedingSequence = false;
        m_vortex.set(-HopperConstants.kFeedSpeed);
    }

    /**
     * Runs the hopper very slowly for testing/verification (bypasses the feed sequence).
     */
    public void runSlow() {
        m_isFeedingSequence = false;
        m_vortex.set(HopperConstants.kCreepSpeed);
    }
}