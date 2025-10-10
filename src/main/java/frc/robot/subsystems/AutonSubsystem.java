package frc.robot.subsystems;

import java.security.PrivateKey;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drivetrain;
import frc.robot.commands.Tester;
import frc.robot.commands.autons.Forward;

public class AutonSubsystem extends SubsystemBase{

    private Drivetrain swerve;
    //private final movement movement = new movement(drivetrain, 0, 0);

    public AutonSubsystem(Drivetrain swerve){
        swerve = this.swerve;
    }
    
    public final Command getCommand(double x, double y, double angle){
        //THIS IS FOR TESTING DON'T KEEP THIS
        Command command = new Tester();

        //Command command = new Forward(swerve, 2);

        return command;
    }
}