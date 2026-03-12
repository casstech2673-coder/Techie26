package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The GameManager acts as the master clock during the match.
 */
public class GameManager extends SubsystemBase {

    private final CommandXboxController m_driver;
    private final CommandXboxController m_operator;

    private static final double RUMBLE_WARNING_TIME = 3.0; // seconds before a shift to rumble

    private String m_hubStatusString = "WAITING...";

    private final SendableChooser<Boolean> m_modeChooser = new SendableChooser<>();

    public GameManager(CommandXboxController driver, CommandXboxController operator) {
        this.m_driver = driver;
        this.m_operator = operator;

        m_modeChooser.setDefaultOption("MATCH MODE", false);
        m_modeChooser.addOption("PRACTICE MODE (Hub Always Active)", true);
        SmartDashboard.putData("Robot Operating Mode", m_modeChooser);
    }

    @Override
    public void periodic() {
        boolean isPracticeMode = m_modeChooser.getSelected() != null && m_modeChooser.getSelected();

        if (isPracticeMode) {
            setRumble(0.0);
            m_hubStatusString = "PRACTICE MODE: Active";
            return;
        }

        if (!DriverStation.isTeleopEnabled()) {
            setRumble(0.0);
            m_hubStatusString = DriverStation.isAutonomousEnabled() ? "AUTO: ACTIVE" : "WAITING FOR TELEOP";
            return;
        }

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) {
            setRumble(0.0);
            m_hubStatusString = "WAITING FOR TELEOP";
            return;
        }

        boolean hubActive = isHubActive();

        // Rumble warning when the hub state is about to change
        Optional<Alliance> alliance = DriverStation.getAlliance();
        String gameData = DriverStation.getGameSpecificMessage();
        if (alliance.isPresent() && gameData != null && !gameData.isEmpty()) {
            char winner = gameData.charAt(0);
            if (winner == 'R' || winner == 'B') {
                boolean redInactiveFirst = (winner == 'R');
                boolean shift1Active = (alliance.get() == Alliance.Red) ? !redInactiveFirst : redInactiveFirst;
                boolean futureHubState = computeHubStateAtTime(matchTime - RUMBLE_WARNING_TIME, shift1Active);
                setRumble(hubActive != futureHubState ? 1.0 : 0.0);
            } else {
                setRumble(0.0);
            }
        } else {
            setRumble(0.0);
        }

        // Update status string for driver dashboard
        if (matchTime <= 30.0) {
            m_hubStatusString = "ENDGAME: BOTH ACTIVE";
        } else if (hubActive) {
            m_hubStatusString = "ACTIVE (SHOOT!)";
        } else {
            m_hubStatusString = "INACTIVE (PASS!)";
        }
    }

    /**
     * Returns whether the hub is currently active for scoring.
     * Respects practice mode, autonomous, and the teleop shift schedule.
     */
    public boolean isHubActive() {
        boolean isPracticeMode = m_modeChooser.getSelected() != null && m_modeChooser.getSelected();
        if (isPracticeMode) return true;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) return false;
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) return true;
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, assume hub is active (likely early in teleop).
        if (gameData == null || gameData.isEmpty()) return true;

        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> { return true; } // Invalid game data — assume active
        }

        // Shift 1 is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red  -> !redInactiveFirst;
            case Blue ->  redInactiveFirst;
        };

        return computeHubStateAtTime(matchTime, shift1Active);
    }

    /**
     * Core shift schedule: returns hub state at any given match time.
     * Extracted so periodic() can also check the state RUMBLE_WARNING_TIME seconds ahead.
     */
    private boolean computeHubStateAtTime(double matchTime, boolean shift1Active) {
        if (matchTime > 130) {
            return true;         // Transition period, hub always active
        } else if (matchTime > 105) {
            return shift1Active; // Shift 1
        } else if (matchTime > 80) {
            return !shift1Active; // Shift 2
        } else if (matchTime > 55) {
            return shift1Active; // Shift 3
        } else if (matchTime > 30) {
            return !shift1Active; // Shift 4
        } else {
            return true;         // Endgame, hub always active
        }
    }

    private void setRumble(double power) {
        m_driver.getHID().setRumble(RumbleType.kBothRumble, power);
        m_operator.getHID().setRumble(RumbleType.kBothRumble, power);
    }

    /**
     * Used by RobotTelemetry to display the current match phase on the dashboard.
     */
    public String getHubStatusString() {
        return m_hubStatusString;
    }
}
