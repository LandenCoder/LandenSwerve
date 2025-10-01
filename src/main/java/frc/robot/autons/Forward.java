package frc.robot.autons;

// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Drivetrain;
// import frc.robot.Robot;

// public class Forward extends SwerveControllerCommand{

//     private static Trajectory generateTrajectory(AutonOrganizer autonOrganizer, Drivetrain swerve){
//         double maxSpeed = swerve.getMaxSpeed();
//         double maxAccel = swerve.getMaxAccel();

//         double speedDivisor = 8;
//         double accelDivisor = 8;

//         double distance = 2;

//         TrajectoryConfig config = new TrajectoryConfig(maxSpeed/speedDivisor, maxAccel/accelDivisor);

//         Pose2d startPose = autonOrganizer.getInitialPose;
//         swerve.setPose(startPose);
//         Pose2d endPose = new Pose2d(
//         startPose.getX() + distance,
//         startPose.getY(), 
//         startPose.getRotation());

//         List<Pose2d> poseList = List.of(startPose, endPose);

//         return TrajectoryGenerator.generateTrajectory(poseList, config);
//     }

//     public Forward(AutonOrganizer autonOrganizer, Drivetrain swerveSubsystem){
//         super(
//             generateTrajectory(autonOrganizer, swerveSubsystem),
//             swerveSubsystem::getPose, 
//             swerveSubsystem.getKinimatics(),
//             swerveSubsystem.getDriveController(),
//             swerveSubsystem::setAutonModuleStates,
//             swerveSubsystem
//         );
//     }

// }
