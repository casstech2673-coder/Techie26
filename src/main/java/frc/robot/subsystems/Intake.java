package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    
    // Hardware
    private final SparkMax m_pivotMotor;
    private final SparkFlex m_rollerMotor; // NEO Vortex
    private final SparkAbsoluteEncoder m_pivotEncoder;
    private final SparkClosedLoopController m_pivotController;

    // Live-tuning: last values applied from SmartDashboard
    private double m_manualSpeedLimit = 0.15;
    private double m_lastPivotP = IntakeConstants.kPivotP;
    private double m_lastPivotI = IntakeConstants.kPivotI;
    private double m_lastPivotD = IntakeConstants.kPivotD;
    private double m_lastPivotMaxOutput = 0.6;

    public Intake() {
        // Initialize Motors
        m_pivotMotor = new SparkMax(IntakeConstants.kPivotMotorId, MotorType.kBrushless);
        m_rollerMotor = new SparkFlex(IntakeConstants.kRollerMotorId, MotorType.kBrushless);

        // Get the absolute encoder and closed-loop controller from the pivot motor
        m_pivotEncoder = m_pivotMotor.getAbsoluteEncoder();
        m_pivotController = m_pivotMotor.getClosedLoopController();

        // ==========================================================
        // PIVOT MOTOR CONFIGURATION
        // ==========================================================
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        
        pivotConfig.inverted(false); // TODO: Change to true if your motor spins backward
        pivotConfig.smartCurrentLimit(40); // Protects the pivot NEO from burning out
        
        // Configure the Absolute Encoder to read in degrees (0 to 360)
        pivotConfig.absoluteEncoder.positionConversionFactor(360.0);
        
        // Configure the PID Controller to use the Absolute Encoder
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(IntakeConstants.kPivotP, IntakeConstants.kPivotI, IntakeConstants.kPivotD)
            .outputRange(-0.6, 0.6); // TODO: Adjust the max speed so it doesn't violently slam during testing

        // Flash the configuration to the motor permanently
        m_pivotMotor.configure(pivotConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Intake/Pivot/kP", IntakeConstants.kPivotP);
        SmartDashboard.putNumber("Intake/Pivot/kI", IntakeConstants.kPivotI);
        SmartDashboard.putNumber("Intake/Pivot/kD", IntakeConstants.kPivotD);
        SmartDashboard.putNumber("Intake/Pivot/MaxOutput", 0.6);
        SmartDashboard.putNumber("Intake/Pivot/ManualSpeed", 0.15);

        // ==========================================================
        // ROLLER MOTOR CONFIGURATION
        // ==========================================================
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        
        rollerConfig.inverted(false); 
        rollerConfig.smartCurrentLimit(50);
        
        m_rollerMotor.configure(rollerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/ArmAngleDeg", getPivotAngle());

        m_manualSpeedLimit = SmartDashboard.getNumber("Intake/Pivot/ManualSpeed", m_manualSpeedLimit);
        double p = SmartDashboard.getNumber("Intake/Pivot/kP", m_lastPivotP);
        double i = SmartDashboard.getNumber("Intake/Pivot/kI", m_lastPivotI);
        double d = SmartDashboard.getNumber("Intake/Pivot/kD", m_lastPivotD);
        double maxOut = SmartDashboard.getNumber("Intake/Pivot/MaxOutput", m_lastPivotMaxOutput);
        if (p != m_lastPivotP || i != m_lastPivotI || d != m_lastPivotD || maxOut != m_lastPivotMaxOutput) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.pid(p, i, d).outputRange(-maxOut, maxOut);
            m_pivotMotor.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kNoPersistParameters);
            m_lastPivotP = p; m_lastPivotI = i; m_lastPivotD = d; m_lastPivotMaxOutput = maxOut;
        }
    }

    /** Zeros the arm relative encoder at the current position. */
    public void zeroPivotEncoder() {
        m_pivotMotor.getEncoder().setPosition(0.0);
    }

    public double getPivotAngle() { return m_pivotEncoder.getPosition(); }
    public double getPivotOutput() { return m_pivotMotor.getAppliedOutput(); }

    // ==========================================================
    // INTAKE COMMAND METHODS
    // ==========================================================

    /**
     * Commands the pivot motor to move to a specific angle using the onboard PID.
     */
    public void setPivotAngle(double targetDegrees) {
        m_pivotController.setSetpoint(targetDegrees, ControlType.kPosition);
    }

    /**
     * Turns the roller motor on.
     */
    public void runRollers() {
        m_rollerMotor.set(IntakeConstants.kIntakeRollerSpeed);
    }

    /**
     * Turns the roller motor off.
     */
    public void stopRollers() {
        m_rollerMotor.set(0.0);
    }
    
    /**
     * Reverses the rollers in case a game piece gets jammed.
     */
    public void reverseRollers() {
        m_rollerMotor.set(-IntakeConstants.kIntakeRollerSpeed);
    }

    /**
     * Drives the pivot open-loop at a very low speed using a joystick input (-1 to 1).
     * Positive input = pivot moves up, negative = pivot moves down.
     * Max output is capped at 15% so nothing moves dangerously fast in test mode.
     */
    public void runPivotManual(double joystickInput) {
        m_pivotMotor.set(joystickInput * m_manualSpeedLimit);
    }

    /**
     * Runs the roller very slowly for testing/verification.
     */
    public void runRollersSlow() {
        m_rollerMotor.set(IntakeConstants.kCreepSpeed);
    }
}