// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import java.security.PublicKey;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final VictorSPX m_turningMotor;
  private final boolean InvertLeft;
  private final boolean InvertBack;
  private final CANEncoder m_driveEncoder;
  private final Encoder m_turningEncoder;
  private double turningEncoderCounts;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  PIDController turningPID = new PIDController(ModuleConstants.kPModuleTurningController, 0, 0);

  /**
   * Constructs a SwerveModule.
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int[] turningEncoderPorts,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed,
      boolean is_invertedLeft,
      boolean is_invertedBack,
      double encoderCounts)
       {
    
    turningEncoderCounts = encoderCounts;
    m_driveMotor = new CANSparkMax (driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.setInverted(is_invertedLeft);
    m_turningMotor = new VictorSPX(turningMotorChannel);

    this.m_driveEncoder = m_driveMotor.getEncoder();
//m_driveEncoder.setInverted(is_invertedLeft);
    this.m_turningEncoder = new Encoder(turningEncoderPorts[0], turningEncoderPorts[1]);

    InvertLeft = is_invertedLeft;
    InvertBack = is_invertedBack;
m_driveEncoder.setPositionConversionFactor(ModuleConstants.kRevolutionsToMeters);
m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kRPMToMetersPerSecond);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -4 and 4 and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    turningPID.enableContinuousInput(-Math.PI, Math.PI);

Shuffleboard.getTab("swere").addNumber("SwerveModouleTurning"+turningMotorChannel, this::getAngleRadians);
    // Shuffleboard.getTab("Swerve").addNumber("SwerveModule"+driveMotorChannel, m_driveEncoder::getPosition);
    Shuffleboard.getTab("Swerve").addNumber("SwerveModule"+driveMotorChannel, this::getDrivePosition);
    resetEncoders();
  }
  public double getConstrictedAngleRadians(double Angle) {
    double temp = Math.signum(Angle);
    Angle = Math.abs(Angle);
    Angle = Angle % Math.PI;
    Angle = Angle * temp;
    // System.out.printf("This is the actual angle %f and caculated is %f", getAngleRadians(), Angle);
    return Angle;
  }
public double getAngleRadians() {
  return (InvertBack? -1.0 : 1.0) * m_turningEncoder.get() * turningEncoderCounts;
  // return (InvertBack? -1 : 1) * m_turningEncoder.get();
  // return (InvertBack? -1 : 1) * m_turningEncoder.get() * 2 * Math.PI / 418;
}

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAngleRadians()));
   // return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.get()));
  }

  public double getDrivePosition() {
    return (InvertLeft? -1 : 1) * m_driveEncoder.getPosition();
  }
  public double getDriveVelocity() {
    return (InvertLeft? -1 : 1) * m_driveEncoder.getVelocity();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean noMovement) {
    if (noMovement) {
//no Movement
m_driveMotor.set(0);
m_turningMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    else {
//There is movement
    

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
    SwerveModuleState.optimize(desiredState, new Rotation2d(getAngleRadians()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = (InvertLeft? -1 : 1) * state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        // m_turningPIDController.calculate(getAngleRadians(), state.angle.getRadians());
        turningPID.calculate(getAngleRadians(), state.angle.getRadians());
    var error = Math.abs((state.angle.getRadians() % (2 * Math.PI)) - (getAngleRadians() % (2 * Math.PI)));
    error = Math.min(error, 2 * Math.PI - error);

    var ready = 1 - error * 2/Math.PI;
    ready = Math.max(ready, 0);
    
    m_driveMotor.set(driveOutput * 1);
    // m_turningMotor.set(VictorSPXControlMode., demand);
    m_turningMotor.set(VictorSPXControlMode.PercentOutput,turnOutput);
  
    // System.out.println(state.angle.getRadians()+","+getAngleRadians()+","+error);
    }
}

  /** Zeros all the SwerveModule encoders. */
  //resetEncoders does not remove field oriented drive
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.reset();
  }
}
