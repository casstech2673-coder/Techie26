package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
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

    // TIMING
    private static final double RUMBLE_WARNING_TIME = 3.0; // Seconds before shift to vibrate

    private boolean m_hubActive = false;
    private boolean m_weWonAuto = false;
    private boolean m_wasTie = false;
    private boolean m_dataReceived = false;
    private String m_hubStatusString = "WAITING...";

    // Mode Switch
    private final SendableChooser<Boolean> m_modeChooser = new SendableChooser<>();

    public GameManager(CommandXboxController driver, CommandXboxController operator) {
        this.m_driver = driver;
        this.m_operator = operator;

        // Set up the Dashboard Switch
        m_modeChooser.setDefaultOption("MATCH MODE", false);
        m_modeChooser.addOption("PRACTICE MODE (Hub Always Active)", true);
        SmartDashboard.putData("Robot Operating Mode", m_modeChooser);
    }

    @Override
    public void periodic() {
        // Check Dashboard Switch
        boolean isPracticeMode = m_modeChooser.getSelected() != null ? m_modeChooser.getSelected() : false;

        // Get Game Data (Who won Auto?)
        if (!m_dataReceived && !isPracticeMode) {
            String gameData = DriverStation.getGameSpecificMessage();
            var myAlliance = DriverStation.getAlliance();

            if (gameData != null && gameData.length() > 0 && myAlliance.isPresent()) {
                m_dataReceived = true; 
                DriverStation.Alliance myColor = myAlliance.get();

                switch (gameData.charAt(0)) {
                    case 'B':
                        m_weWonAuto = (myColor == DriverStation.Alliance.Blue);
                        m_wasTie = false;
                        break;
                    case 'R':
                        m_weWonAuto = (myColor == DriverStation.Alliance.Red);
                        m_wasTie = false;
                        break;
                    case 'T':
                        m_weWonAuto = false;
                        m_wasTie = true;
                        break;
                    default:
                        m_weWonAuto = false;
                        m_wasTie = false;
                        break;
                }
            }
        }

        double matchTime = DriverStation.getMatchTime(); 

        // Override for practice mode
        if (isPracticeMode) {
            setRumble(0.0);
            m_hubStatusString = "PRACTICE MODE: 'Active'";
            return;
        }

        //  NORMAL MATCH LOGIC
        if (!DriverStation.isTeleopEnabled() || matchTime < 0) {
            setRumble(0.0);
            m_hubStatusString = "WAITING FOR TELEOP";
            return;
        }

        m_hubActive = calculateHubState(matchTime, m_weWonAuto, m_wasTie);

        // Predict the Future (Rumble Logic)
        boolean futureHubState = calculateHubState(matchTime - RUMBLE_WARNING_TIME, m_weWonAuto, m_wasTie);
        if (m_hubActive != futureHubState) {
            setRumble(1.0); 
        } else {
            setRumble(0.0); 
        }

        // Update the string so Telemetry can read it later
        if (matchTime <= 30.0) {
            m_hubStatusString = "ENDGAME: BOTH ACTIVE";
        } else if (m_hubActive) {
            m_hubStatusString = "ACTIVE (SHOOT!)";
        } else {
            m_hubStatusString = "INACTIVE (PASS!)";
        }
    }

    private boolean calculateHubState(double timeRemaining, boolean wonAuto, boolean wasTie) {
        if (timeRemaining > 130.0 || timeRemaining <= 30.0) return true; 

        boolean isShift1 = (timeRemaining <= 130.0 && timeRemaining > 105.0);
        boolean isShift2 = (timeRemaining <= 105.0 && timeRemaining > 80.0);
        boolean isShift3 = (timeRemaining <= 80.0 && timeRemaining > 55.0);
        boolean isShift4 = (timeRemaining <= 55.0 && timeRemaining > 30.0);

        boolean amIWinner = wonAuto && !wasTie;
        boolean amILoser = !wonAuto || wasTie; 

        if (isShift1 || isShift3) {
            if (amIWinner) return false;
            if (amILoser) return true;
        } else if (isShift2 || isShift4) {
            if (amIWinner) return true;
            if (amILoser) return false;
        }

        return false; 
    }

    private void setRumble(double power) {
        m_driver.getHID().setRumble(RumbleType.kBothRumble, power);
        m_operator.getHID().setRumble(RumbleType.kBothRumble, power);
    }

    /**
     * Checks if the Hub is active. Automatically respects the Dashboard Switch
     */
    public boolean isHubActive() {
        boolean isPracticeMode = m_modeChooser.getSelected() != null ? m_modeChooser.getSelected() : false;
        if (isPracticeMode) {
            return true;
        }
        return m_hubActive;
    }

    /**
     * Used by the Telemetry subsystem to display the current match phase on the dashboard.
     */
    public String getHubStatusString() {
        return m_hubStatusString;
    }
}