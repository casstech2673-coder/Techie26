package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    // Hardware
    private final SparkMax m_pivotMotor;
    private final SparkMax m_rollerMotor;
    // Relative encoder: zeroed at startup (arm manually set to stowed position before enabling)
    private final RelativeEncoder m_pivotEncoder;
    private final SparkClosedLoopController m_pivotController;

    // Live-tuning state
    private double m_manualSpeedLimit = 0.3;
    private double m_lastPivotP = IntakeConstants.kPivotP;
    private double m_lastPivotI = IntakeConstants.kPivotI;
    private double m_lastPivotD = IntakeConstants.kPivotD;
    private double m_lastPivotMaxOutput = 0.6;

    public Intake() {
        m_pivotMotor = new SparkMax(IntakeConstants.kPivotMotorId, MotorType.kBrushless);
        m_rollerMotor = new SparkMax(IntakeConstants.kRollerMotorId, MotorType.kBrushless);

        m_pivotEncoder = m_pivotMotor.getEncoder();
        m_pivotController = m_pivotMotor.getClosedLoopController();

        // ==========================================================
        // PIVOT MOTOR CONFIGURATION
        // ==========================================================
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.inverted(true);           // Motor was physically inverted; fix here
        pivotConfig.smartCurrentLimit(40);
        pivotConfig.idleMode(IdleMode.kCoast); // Coast when disabled — arm movable by hand

        // Use the built-in relative encoder (zeroed at startup)
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(IntakeConstants.kPivotP, IntakeConstants.kPivotI, IntakeConstants.kPivotD)
            .outputRange(-0.6, 0.6);

        m_pivotMotor.configure(pivotConfig,
            com.revrobotics.ResetMode.kResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Intake/Pivot/kP", IntakeConstants.kPivotP);
        SmartDashboard.putNumber("Intake/Pivot/kI", IntakeConstants.kPivotI);
        SmartDashboard.putNumber("Intake/Pivot/kD", IntakeConstants.kPivotD);
        SmartDashboard.putNumber("Intake/Pivot/MaxOutput", 0.6);
        SmartDashboard.putNumber("Intake/Pivot/ManualSpeed", 0.15);

        // ==========================================================
        // ROLLER MOTOR CONFIGURATION
        // ==========================================================
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.inverted(true);
        rollerConfig.smartCurrentLimit(40);
        m_rollerMotor.configure(rollerConfig,
            com.revrobotics.ResetMode.kResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        m_manualSpeedLimit = SmartDashboard.getNumber("Intake/Pivot/ManualSpeed", m_manualSpeedLimit);
        double p      = SmartDashboard.getNumber("Intake/Pivot/kP",        m_lastPivotP);
        double i      = SmartDashboard.getNumber("Intake/Pivot/kI",        m_lastPivotI);
        double d      = SmartDashboard.getNumber("Intake/Pivot/kD",        m_lastPivotD);
        double maxOut = SmartDashboard.getNumber("Intake/Pivot/MaxOutput", m_lastPivotMaxOutput);
        if (p != m_lastPivotP || i != m_lastPivotI || d != m_lastPivotD || maxOut != m_lastPivotMaxOutput) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.pid(p, i, d).outputRange(-maxOut, maxOut);
            m_pivotMotor.configure(config,
                com.revrobotics.ResetMode.kNoResetSafeParameters,
                com.revrobotics.PersistMode.kNoPersistParameters);
            m_lastPivotP = p; m_lastPivotI = i; m_lastPivotD = d; m_lastPivotMaxOutput = maxOut;
        }
    }

    /** Switches pivot to Coast mode — call when robot is disabled so arm can be moved freely. */
    public void setCoastMode() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        m_pivotMotor.configure(config,
            com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kNoPersistParameters);
    }

    /** Switches pivot to Brake mode — call on auto/teleop init so arm holds its position. */
    public void setBrakeMode() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        m_pivotMotor.configure(config,
            com.revrobotics.ResetMode.kNoResetSafeParameters,
            com.revrobotics.PersistMode.kNoPersistParameters);
    }

    /** Zeros the arm relative encoder at the current position. */
    public void zeroPivotEncoder() {
        m_pivotEncoder.setPosition(0.0);
    }

    public double getPivotPosition() { return m_pivotEncoder.getPosition(); }
    public double getPivotOutput()   { return m_pivotMotor.getAppliedOutput(); }
    public double getPivotCurrent()  { return m_pivotMotor.getOutputCurrent(); }

    // ==========================================================
    // INTAKE COMMAND METHODS
    // ==========================================================

    /** Commands the pivot to a target position (motor rotations) using the onboard PID. */
    public void setPivotPosition(double targetRotations) {
        m_pivotController.setSetpoint(targetRotations, ControlType.kPosition);
    }

    public void runRollers() {
        m_rollerMotor.set(IntakeConstants.kIntakeRollerSpeed);
    }

    public void stopRollers() {
        m_rollerMotor.set(0.0);
    }

    public void reverseRollers() {
        m_rollerMotor.set(-IntakeConstants.kIntakeRollerSpeed);
    }

    /** Open-loop pivot control via joystick input (-1 to 1). Max output is m_manualSpeedLimit. */
    public void runPivotManual(double joystickInput) {
        m_pivotMotor.set(joystickInput * m_manualSpeedLimit);
    }

    public void runRollersSlow() {
        m_rollerMotor.set(IntakeConstants.kCreepSpeed);
    }
}
