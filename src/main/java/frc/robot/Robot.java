// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.reset.GyroResetCommand;
import edu.wpi.first.wpilibj.PS4Controller.Button;

public class Robot extends LoggedRobot {
  public final CommandXboxController controller = new CommandXboxController(0);
  public final Drivetrain swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  private double speedDivisor;

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    swerve.updateOdometry();
  }

  @Override
  public void teleopInit() {
    swerve.resetOdometry();
    controller.leftBumper().toggleOnTrue(swerve.sysIdQuadistaticForwards());
    controller.rightBumper().toggleOnTrue(swerve.sysIdQuadistaticBackwards());
    controller.y().onTrue(new GyroResetCommand(swerve));
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
    swerve.periodic();
    robotPeriodic();
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return



    // negative values when we push forward.

    // if (controller.getAButton()) {
    //   swerve.resetGyro();
    // }
    // if (controller.getStartButton() && controller.getLeftBumperButton() && controller.getRightBumperButton()) {
    //   speedDivisor = 1;
    // } else {
    //   speedDivisor = 9;
    // }
    //}

    // while (controller.getLeftBumperButtonPressed()) {
    //   swerve.sysIdDynamicForwards();
    // }
    // while (controller.getRightBumperButtonPressed()) {
    //   swerve.sysIdDynamicBackwards();
    // }

    final var xSpeed = -xspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.05))
        * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -yspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.05))
        * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), 0.05))
        * Drivetrain.kMaxAngularSpeed;

    //swerve.drive(xSpeed/speedDivisor, ySpeed/speedDivisor, rot/speedDivisor, fieldRelative, getPeriod());
    // swerve.drive(xSpeed/9, 0, 0, fieldRelative, getPeriod());
  }

  @Override
  public void disabledPeriodic() {
    swerve.periodic();
  }

  @Override
  public void robotInit() {

    /* What to do when running the robot normally */
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs")); // Log to roboRIO for download
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }
    /*
     * What to do when making a replay log (changing a real log into a simulated
     * log)
     */
    else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    /* Unofficial Rev Compatible Logger, used to log Rev data */
    Logger.registerURCL(URCL.startExternal());

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
  }
}
