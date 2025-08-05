// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private static final double WHEEL_RADIUS = Units.inchesToMeters(0.985);
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;
  private static final double GEAR_RATIO = 7.2;
  public static final double DRIVE_METERS_PER_MOTOR_ROTATION = WHEEL_RADIUS * Math.PI * 33 / 45 / 15/13;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANcoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.000, 0, 0);

  private final PIDController m_turningPIDController = new PIDController(
      0.02,
      0,
      0.0001);

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorID           PWM output for the drive motor.
   * @param turningMotorID         PWM output for the turning motor.
   * @param driveEncoderID         DIO input for the drive encoder channel A
   * @param driveEncoderChannelB   DIO input for the drive encoder channel B
   * @param turningEncoderID       DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int turningEncoderID,
      double turningOffset,
      boolean driveInverted,
      boolean turningInverted) {
    m_driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = new CANcoder(turningEncoderID);

    MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
    magnetConfig.MagnetOffset = turningOffset;
    m_turningEncoder.getConfigurator().apply(magnetConfig);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig turnConfig = new SparkMaxConfig();

    driveConfig.encoder
        .positionConversionFactor(DRIVE_METERS_PER_MOTOR_ROTATION);
    driveConfig.encoder.velocityConversionFactor(
        DRIVE_METERS_PER_MOTOR_ROTATION / 60);

    driveConfig.idleMode(IdleMode.kBrake);

    turnConfig.inverted(turningInverted);

    m_driveMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius /
    // kEncoderResolution);

    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-180, 180);
    //m_turningPIDController.reset(getAngle());
  }

  /**
   * 
   * @return
   *         angle, in degrees
   */
  public double getAngle() {
    return m_turningEncoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = Rotation2d.fromDegrees(getAngle());

    // Optimize the reference state to avoid spinning further than 90 degrees

    // desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired
    // direction of travel that can occur when modules change directions. This
    // results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.

    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
        desiredState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getAngle(), desiredState.angle.getDegrees());

    //final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput);// + turnFeedforward

    SmartDashboard.putNumber("Voltage", turnOutput);
    SmartDashboard.putNumber("getAngle()", getAngle());
    SmartDashboard.putNumber("getDegrees", desiredState.angle.getDegrees());
  }
}