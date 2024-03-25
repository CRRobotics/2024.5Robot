package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Winch;
import frc.robot.util.Constants;

public class Extend extends Command implements Constants.Winch
{
    private Winch winch;
    private long startTime;
    private boolean finished;

    public Extend(Winch winch) {
        this.winch = winch;
        addRequirements(winch);
        finished = false;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        winch.setSpeed(extendSpeed);
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() > startTime + SmartDashboard.getNumber("winch/extend time", extendTime)) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        winch.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}