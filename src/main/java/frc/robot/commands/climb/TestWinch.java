package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Winch;
import frc.robot.util.Constants;

public class TestWinch extends Command implements Constants.Winch
{
    Winch winch; 

    public TestWinch(Winch winch) {
        this.winch = winch;
        addRequirements(winch);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        winch.setSpeed(0.1);
        //winch.getPosition();
        //winch.setPosition(0);
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
