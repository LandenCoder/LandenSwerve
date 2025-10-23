package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Drivetrain;
import frc.robot.Robot;

public class Forward extends SwerveControllerCommand{

    private static Trajectory generateTrajectory(Drivetrain swerve, double metersForward){
        double maxSpeed = swerve.getMaxSpeed();
        double maxAccel = swerve.getMaxAccel();

        double speedDivisor = 9;
        double accelDivisor = 18;

        double distance = metersForward;

        TrajectoryConfig config = new TrajectoryConfig(maxSpeed/speedDivisor, maxAccel/accelDivisor);

        Pose2d startPose = new Pose2d(0.0, 0.0, swerve.returnRotation2d());
        swerve.setPose(startPose);
        Pose2d endPose = new Pose2d(
        startPose.getX() + distance,
        startPose.getY(), 
        startPose.getRotation());

        List<Pose2d> poseList = List.of(startPose, endPose);

        return TrajectoryGenerator.generateTrajectory(poseList, config);
    }

    public Forward(Drivetrain swerve, double metersForward){
        super(
            generateTrajectory(swerve, metersForward),
            swerve::getPose2d, 
            swerve.getKinimatics(),
            swerve.getDriveController(),
            swerve::setAutonModuleStates,
            swerve
        );
    }
}