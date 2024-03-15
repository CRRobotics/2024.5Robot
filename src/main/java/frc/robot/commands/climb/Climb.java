package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ActivityState;
import frc.robot.RobotContainer.ControlState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Winch;
import frc.robot.util.Constants;

public class Climb extends Command implements Constants.Winch
{
    Winch winch;
    private long startTime;
    private boolean initialRun;
    private boolean leftFinished;
    private boolean rightFinished;
    private long extendStopTime;
    private long liveTime;
    private boolean finished;
    // long retractStopTime;
    private enum ClimbSpeed{Fast, Slow};

    public Climb(Winch winch, Shooter shooter) {
        this.winch = winch;
        addRequirements(winch);
        addRequirements(shooter);
        // initialRun = true;
        // leftFinished = false;
        // rightFinished = false;
        // SmartDashboard.putNumber("winch/extend speed", extendSpeed);
        // SmartDashboard.putNumber("winch/retract speed", retractSpeed);
        // SmartDashboard.putNumber("winch/extendTime", extendTime);
        // SmartDashboard.putNumber("winch/currentDifferenceThreshold", currentDifferenceThreshold);
    }

    @Override
    public void initialize() {
        RobotContainer.activityState = ActivityState.CLIMBING;
        finished = false;
        // if (initialRun) {
        startTime = System.currentTimeMillis();
            // initialRun = false;
            // liveTime = 0;
        // } else {
            // startTime = System.currentTimeMillis();
        // }
        extendStopTime = startTime + (long)extendTime;// (long)SmartDashboard.getNumber("winch/extendTime", extendTime);
    }

    /*
     * Moves the winch, changing speeds after a set time, and stopping if it detects a significant current difference
     */
    @Override
    public void execute() {
        // if (Winch.leftSwitch.isPressed()) {
        //     winch.setLeftSpeed(0);
        //     leftFinished = true;
        //     System.out.println("left climb limit switch pressed");
        //     return;
        // }
        // if (Winch.rightSwitch.isPressed()) {
        //     winch.setRightSpeed(0);
        //     rightFinished = true;
        //     System.out.println("right climb limit switch pressed");
        //     return;
        // }
        // if (Math.abs(winch.getCurrentDifference()/2) >= SmartDashboard.getNumber("winch/currentDifferenceThreshold", currentDifferenceThreshold)) {
        //     System.out.println("lbbuehhel");
        //     winch.setSpeed(0);
        //     finished = true;
        //     return;
        // } else 
        if (System.currentTimeMillis() < extendStopTime) {
            winch.setSpeed(extendSpeed);// SmartDashboard.getNumber("winch/extend speed", extendSpeed));
        } else {
            winch.setSpeed(retractSpeed);//SmartDashboard.getNumber("winch/retract speed", retractSpeed));
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.activityState = ActivityState.IDLE;
        liveTime = System.currentTimeMillis() - startTime;
        winch.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}