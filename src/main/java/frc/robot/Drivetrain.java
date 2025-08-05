// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  // TODO: measure into actual locations
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(8, 1, 0, 0.34326, false, false);
  private final SwerveModule m_frontRight = new SwerveModule(2, 3, 1, 0.29980, false, false);
  private final SwerveModule m_backLeft = new SwerveModule(6, 7, 3, 0.26367, false, false);
  private final SwerveModule m_backRight = new SwerveModule(4, 5, 2, 0.31396, false, false);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final AHRS m_gyro;

  private final SwerveDriveOdometry m_odometry;
  private Field2d m_field = new Field2d();

  public Drivetrain() {
    m_gyro = new AHRS(NavXComType.kUSB1);

    m_gyro.reset();

    SmartDashboard.putData("Field", m_field);

    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Swerve Module/fl angle", m_frontLeft.getAngle());
    SmartDashboard.putNumber("Swerve Module/fr angle", m_frontRight.getAngle());
    SmartDashboard.putNumber("Swerve Module/bl angle", m_backLeft.getAngle());
    SmartDashboard.putNumber("Swerve Module/br angle", m_backRight.getAngle());

    SmartDashboard.putNumber("Swerve Module/fl speed", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve Module/fr speed", m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve Module/bl speed", m_backLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve Module/br speed", m_backRight.getState().speedMetersPerSecond);

    SmartDashboard.putNumber("gyro", m_gyro.getAngle());

    // do this later (maybe)

    // SmartDashboard.putNumber("Swerve Module/fl desired speed",
    // m_frontLeft.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("Swerve Module/fr desired speed",
    // m_frontRight.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("Swerve Module/bl desired speed",
    // m_backLeft.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("Swerve Module/br desired speed",
    // m_backRight.getState().speedMetersPerSecond);

    // SmartDashboard.putData("Field", m_field);
    
    updateOdometry();
    // m_field.setRobotPose(m_odometry.getPoseMeters());
    super.periodic();
    
  }

}
