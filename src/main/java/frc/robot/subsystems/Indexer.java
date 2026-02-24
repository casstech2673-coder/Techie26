// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// REVLib Imports
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  /* It's best practice to put these in the Constants.java file, but they are here for easy reading. */
  private static final int INDEXER_MOTOR_CAN_ID = 0; //TODO: Change to your actual CAN ID
  private static final double FEED_SPEED = 0.8; // 80% power, tune as needed
  private static final double REVERSE_SPEED = -0.3; // Slower speed for unjamming

  // Hardware
  private final SparkFlex indexerMotor;
  private final SparkFlexConfig motorConfig;

  /** Creates a new Indexer. */
  public Indexer() {
    // Initialize Spark Flex Controller
    indexerMotor = new SparkFlex(INDEXER_MOTOR_CAN_ID, MotorType.kBrushless);

    // Create a new configuration object
    motorConfig = new SparkFlexConfig();

    // Write configuration settings
    motorConfig.inverted(false) // Invert here if the motor spins backwards
               .idleMode(IdleMode.kCoast); // Coast so the motor free spins when off

    // Apply configuration
    indexerMotor.configure(motorConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }

  /**
   * Runs the indexer motor to feed the shooter.
   */
  public void runIndexer() {
    indexerMotor.set(FEED_SPEED);
  }

  /**
   * Reverses the indexer (might be useful for unjamming).
   */
  public void reverseIndexer() {
    indexerMotor.set(REVERSE_SPEED);
  }

  /**
   * Stops the indexer motor.
   */
  public void stop() {
    indexerMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}