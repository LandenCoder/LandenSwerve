package frc.robot.reset;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

public class GyroResetCommand extends Command {
    private Drivetrain drivetrain;

    public GyroResetCommand(Drivetrain drivetrain){
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }   
}