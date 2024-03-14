package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Winch;
import frc.robot.util.Constants;

public class Retract extends Command implements Constants.Winch
{
    Winch winch;

    public Retract(Winch winch) {
        this.winch = winch;
        addRequirements(winch);
    }

    @Override
    public void initialize() {
        winch.setSpeed(retractSpeed);
    }

    @Override
    public void execute() {
        winch.setSpeed(retractSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        winch.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}