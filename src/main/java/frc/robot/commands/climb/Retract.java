package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Winch;

public class Extend extends CommandBase
{
    public Retract(Winch wnch)
    {
        this.wnch = wnch;
        addRequirements(wnch);
    }

    @Override
    public void initialize() {
        wnch.setSpeed(0.6);
    }

    @Override
    public void execute()
    {
        wnch.setSpeed(0.6);
    }

    @Override
    public void end(boolean interrupted) {
        wnch.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}