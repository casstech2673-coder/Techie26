package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {
    private final SwerveDrive m_swerve;
    
    // Suppliers for joysticks
    private final DoubleSupplier m_xSpdFunction;
    private final DoubleSupplier m_ySpdFunction;
    private final DoubleSupplier m_turnSpdFunction;
    private final BooleanSupplier m_fieldRelativeFunction;
    
    // Suppliers for Buttons (A, B, X, Y)
    private final BooleanSupplier m_aButton;
    private final BooleanSupplier m_bButton;
    private final BooleanSupplier m_xButton;
    private final BooleanSupplier m_yButton;

    // Joystick Deadband (Tune this if your controllers are old/drifty)
    private static final double DEADBAND = 0.05;

    public TeleopDrive(
            SwerveDrive swerve,
            DoubleSupplier xSpdFunction,
            DoubleSupplier ySpdFunction,
            DoubleSupplier turnSpdFunction,
            BooleanSupplier fieldRelativeFunction,
            BooleanSupplier aButton,
            BooleanSupplier bButton,
            BooleanSupplier xButton,
            BooleanSupplier yButton) {

        this.m_swerve = swerve;
        this.m_xSpdFunction = xSpdFunction;
        this.m_ySpdFunction = ySpdFunction;
        this.m_turnSpdFunction = turnSpdFunction;
        this.m_fieldRelativeFunction = fieldRelativeFunction;
        this.m_aButton = aButton;
        this.m_bButton = bButton;
        this.m_xButton = xButton;
        this.m_yButton = yButton;

        // Require the swerve subsystem so no other command can drive at the same time
        addRequirements(m_swerve);
    }

    @Override
    public void execute() {
        // Get Joystick inputs and apply deadbands
        // Note: Joysticks are negative when pushed forward/left, so we usually invert them here.
        double xInput = -MathUtil.applyDeadband(m_xSpdFunction.getAsDouble(), DEADBAND);
        double yInput = -MathUtil.applyDeadband(m_ySpdFunction.getAsDouble(), DEADBAND);
        double turnInput = -MathUtil.applyDeadband(m_turnSpdFunction.getAsDouble(), DEADBAND);

        // Alliance Flipping (Keep "Forward" pointing away from the driver)
        boolean isRedAlliance = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            isRedAlliance = true;
        }

        if (isRedAlliance && m_fieldRelativeFunction.getAsBoolean()) {
            xInput = -xInput;
            yInput = -yInput;
        }

        // Face Buttons (A, B, X, Y) for Heading Snap
        boolean isSnapping = true;
        Rotation2d targetAngle = new Rotation2d();

        // Standard spatial mapping for the buttons
        if (m_yButton.getAsBoolean()) {
            targetAngle = Rotation2d.fromDegrees(0); // Y (Top): Away from driver
        } else if (m_bButton.getAsBoolean()) {
            targetAngle = Rotation2d.fromDegrees(-90); // B (Right): Right
        } else if (m_aButton.getAsBoolean()) {
            targetAngle = Rotation2d.fromDegrees(180); // A (Bottom): Toward driver
        } else if (m_xButton.getAsBoolean()) {
            targetAngle = Rotation2d.fromDegrees(90); // X (Left): Left
        } else {
            // No face buttons are being pressed
            isSnapping = false; 
        }

        // Drive robot
        if (isSnapping) {
            // Drive using our custom Heading Snap method to automatically spin to the target
            m_swerve.driveWithHeading(xInput, yInput, targetAngle);
        } else {
            // Drive normally using the right joystick for rotation
            m_swerve.drive(xInput, yInput, turnInput, m_fieldRelativeFunction.getAsBoolean());
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot if the command ends for safety
        m_swerve.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false; // Default drive command never finishes unless interrupted
    }
}