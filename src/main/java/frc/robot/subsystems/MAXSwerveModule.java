// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;

import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSpark;
  private final SparkFlex m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AnalogEncoder m_turningEncoder; // Only use the external analog encoder

  private final SparkClosedLoopController m_drivingClosedLoopController;
  
  // Software PID controller for turning since we're using external encoder
  private final PIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration uses only the external
   * analog absolute encoder for turning control.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, int analogEncoderPort) {
    m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkFlex(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    
    // Create analog encoder using the specified analog port
    m_turningEncoder = new AnalogEncoder(analogEncoderPort, 2 * Math.PI, 0.0);

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    
    // Create software PID controller for turning
    // These gains will need to be tuned for your robot
    m_turningPIDController = new PIDController(1.0, 0.0, 0.0);
    m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);

    // Apply configurations - only need driving config since turning is software controlled
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    // Simple turning motor config - no closed loop since we're doing it in software
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.get() - m_chassisAngularOffset);
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to get position relative to the chassis
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),new Rotation2d(m_turningEncoder.get() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to get position relative to the chassis
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.get() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.get()));

    // Command driving motor using hardware PID
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    
    // Command turning motor using software PID
    double currentPosition = m_turningEncoder.get();
    double targetPosition = correctedDesiredState.angle.getRadians();
    
    // Calculate PID output
    double pidOutput = m_turningPIDController.calculate(currentPosition, targetPosition);
    
    // Clamp output to [-1, 1] and apply to motor
    pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
    m_turningSpark.set(pidOutput);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
    // Note: Analog encoders are absolute and don't need to be reset
    // The chassis angular offset handles the zero position
  }
  
  /**
   * Get the turning PID controller for tuning.
   * @return The turning PID controller
   */
  public PIDController getTurningPIDController() {
    return m_turningPIDController;
  }

  public void resetToAbsolute() {
    double absoluteAngle = m_turningEncoder.get(); 
    double correctedAngle = absoluteAngle - m_chassisAngularOffset;
    correctedAngle = MathUtil.inputModulus(correctedAngle, 0, 2 * Math.PI);
    m_desiredState = new SwerveModuleState(0.0, new Rotation2d(0));
    double turnOutput = m_turningPIDController.calculate(correctedAngle, 0.0);
    m_turningSpark.set(turnOutput);
    System.out.println("Resetting modules to absolute...");
  }
}