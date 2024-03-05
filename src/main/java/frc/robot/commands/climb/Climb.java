package frc.robot.commands.climb;

import java.util.Arrays;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Winch;
import frc.robot.util.Constants;

public class Climb extends Command implements Constants.Winch
{
    Winch winch;
    long startTime;
    boolean initialRun;
    boolean finished;
    long extendStopTime;
    long liveTime;
    // long retractStopTime;

    public Climb(Winch winch, Shooter shooter) {
        this.winch = winch;
        addRequirements(winch);
        addRequirements(shooter);
        initialRun = true;
        finished = false;
        SmartDashboard.putNumber("winch/extend speed", extendSpeed);
        SmartDashboard.putNumber("winch/retract speed", retractSpeed);
    }

    @Override
    public void initialize() {
        if (initialRun) {
            startTime = System.currentTimeMillis();
            initialRun = false;
            liveTime = 0;
        } else {
            startTime = System.currentTimeMillis() - liveTime;
        }
        extendStopTime = startTime + (long)extendTime;
    }

    /*
     * TODO:
     * - Freeze all other subsystems when climibing
     *   - Use a timer to make a cancellable period within which it can be cancelled, but after which you are locked in
     *   - Be sure to reset timers in this case (but keep track of how long it was running for)
     * - Make cancellable if the driver misclicked
     * - Add ability to automatically stop the winch if only one hook catches
     */
    @Override
    public void execute() {
        if (Math.abs((winch.getCurrentDifference()/2)) >= currentDifferenceThreshold) {
            winch.setSpeed(0);
            finished = true;
            return;
        } else if (System.currentTimeMillis() < extendStopTime) {
            winch.setSpeed(SmartDashboard.getNumber("winch/extend speed", extendSpeed));
        } else {
            winch.setSpeed(SmartDashboard.getNumber("winch/retract speed", retractSpeed));
        }
    }

    @Override
    public void end(boolean interrupted) {
        liveTime = System.currentTimeMillis() - startTime;
        winch.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}