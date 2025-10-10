package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

public class Tester extends Command{
    private Drivetrain swerve;

    public void tester(){

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.makeMeHappy();
    }

    @Override
    public boolean isFinished() {
        return true;
    }   
}
