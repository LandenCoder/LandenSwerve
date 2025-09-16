// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.PrintStream;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  // TODO: measure into actual locations
  private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule frontLeft = new SwerveModule(8, 1, 0, 0.35181, false, false);
  private final SwerveModule frontRight = new SwerveModule(2, 3, 1, 0.29321, false, false);
  private final SwerveModule backLeft = new SwerveModule(6, 7, 3, 0.26343, false, false);
  private final SwerveModule backRight = new SwerveModule(4, 5, 2, -0.43896, false, false);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final AHRS gyro;

  private final SwerveDriveOdometry odometry;
  private Field2d field = new Field2d();

  private final Constraints autonRotationConstraints = new Constraints(Math.PI * 4, Math.PI * 2);
  private final PIDController autonDrivePIDController = new PIDController(0.8, 0, 0); // 2
  private final ProfiledPIDController autonThetaController = new ProfiledPIDController(3, 0, 0,
      autonRotationConstraints); // 1.5

      

  // Create the SysId routine
  private SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, null, null, // Use default config
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
      new SysIdRoutine.Mechanism(
          (voltage) -> voltageDrive(voltage.in(Volts)),
          // (voltage) -> voltageDrive(voltage.in(Volts)),
          null, // No log consumer, since data is recorded by AdvantageKit
          this));
  
  public Drivetrain() {
    gyro = new AHRS(NavXComType.kUSB1);

    gyro.reset();

    SmartDashboard.putData("Field", field);

    odometry = new SwerveDriveOdometry(
        kinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    // field.setRobotPose(odometry.getPoseMeters());
    field.setRobotPose(odometry.getPoseMeters());

    // The methods below return Command objects
  }

  public Command sysIdQuadistaticForwards() {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdQuadistaticBackwards() {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdDynamicForwards() {
    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdDynamicBackwards() {
    return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
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
    var swerveModuleStates = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }
/** 
 * @param voltage voltage
 */
  public void voltageDrive(Double voltage) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            true
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    voltage, 0, 0, gyro.getRotation2d())
                : new ChassisSpeeds(0.1, 0, 0),
            0.1));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
     12);//ITS A VOLTAGE!!!

    frontLeft.sysidTestVoltage(swerveModuleStates[0]);
    frontRight.sysidTestVoltage(swerveModuleStates[1]);
    backLeft.sysidTestVoltage(swerveModuleStates[2]);
    backRight.sysidTestVoltage(swerveModuleStates[3]);
  }
  public void sysidtester(Double voltage){
    System.out.println("hi");
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void resetOdometry() {
    frontLeft.resetOdometry();
    frontRight.resetOdometry();
    backLeft.resetOdometry();
    backRight.resetOdometry();
  }

  public HolonomicDriveController getDriveController() {
    return new HolonomicDriveController(
        autonDrivePIDController,
        autonDrivePIDController,
        autonThetaController);
  }

  /*
   * @return the drive kinematics.
   */
  public SwerveDriveKinematics getKinimatics() {
    return kinematics;
  }

  /*
   * @returns max speed
   */
  public double getMaxSpeed() {
    return kMaxSpeed;
  }

  //TODO: make this
  // public double getMaxAccel() {
  // return ;
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyro", gyro.getAngle());

    SmartDashboard.putNumber("robotX", field.getRobotPose().getX());
    SmartDashboard.putNumber("robotY", field.getRobotPose().getY());

    SmartDashboard.putNumber("MetersDriven/fl", frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("MetersDriven/fr", frontRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("MetersDriven/bl", backLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("MetersDriven/br", backRight.getPosition().distanceMeters);

    // do this later (maybe)
    // SmartDashboard.putNumber("Swerve Module/fl desired speed",
    // frontLeft.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("Swerve Module/fr desired speed",
    // frontRight.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("Swerve Module/bl desired speed",
    // backLeft.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("Swerve Module/br desired speed",
    // backRight.getState().speedMetersPerSecond);

    updateOdometry();
    SmartDashboard.putData("Field", field);
    field.setRobotPose(odometry.getPoseMeters());

    Logger.recordOutput("FLmoduleEncoder", frontLeft.getAngle());
    Logger.recordOutput("FRmoduleEncoder", frontRight.getAngle());
    Logger.recordOutput("BLmoduleEncoder", backLeft.getAngle());
    Logger.recordOutput("BRmoduleEncoder", backRight.getAngle());
  }
}
//API