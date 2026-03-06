package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class TuneShooter extends Command {
    
    private final Shooter m_shooter;
    private final CommandXboxController m_operator;
    
    // Starting defaults
    private double m_testRps = 40.0;
    private double m_testHood = 2.0;

    public TuneShooter(Shooter shooter, CommandXboxController operator) {
        this.m_shooter = shooter;
        this.m_operator = operator;
        
        // This command TAKES OVER the shooter, temporarily ignoring the Superstructure
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        int pov = m_operator.getHID().getPOV();

        // 50 times a second, we apply small changes if a D-Pad button is held
        if (pov == 0)   m_testRps += 0.2;   // D-Pad UP: Increase Flywheel Speed
        if (pov == 180) m_testRps -= 0.2;   // D-Pad DOWN: Decrease Flywheel Speed
        if (pov == 90)  m_testHood += 0.05; // D-Pad RIGHT: Increase Hood Angle (Aim Higher)
        if (pov == 270) m_testHood -= 0.05; // D-Pad LEFT: Decrease Hood Angle (Aim Lower)

        // Limits
        m_testRps = Math.max(0.0, Math.min(m_testRps, 110.0)); // TODO: Max 110 RPS
        m_testHood = Math.max(0.0, Math.min(m_testHood, 12.0)); // TODO: Max 12 Rotations

        // Apply to the motors
        m_shooter.setFlywheelVelocity(m_testRps);
        m_shooter.setHoodPosition(m_testHood);

        // ONLY push the live test numbers to the dashboard
        SmartDashboard.putNumber("TUNING/Current Test RPS", m_testRps);
        SmartDashboard.putNumber("TUNING/Current Test Hood (Rot)", m_testHood);
    }

    @Override
    public void end(boolean interrupted) {
        // As soon as the operator lets go of the button, shut down the shooter
        m_shooter.stopShooter();
    }
}