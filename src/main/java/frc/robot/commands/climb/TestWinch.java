package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ActivityState;
import frc.robot.RobotContainer.ControlState;
import frc.robot.subsystems.Winch;
import frc.robot.util.Constants;

public class TestWinch extends Command implements Constants.Winch
{
    Winch winch; 

    public TestWinch(Winch winch) {
        this.winch = winch;
        addRequirements(winch);
        SmartDashboard.putNumber("winch motor speed", 0);
    }

    @Override
    public void initialize() {
        RobotContainer.activityState = ActivityState.CLIMBING;
        if (!DriverStation.isAutonomous()) RobotContainer.controlState = ControlState.MANUAL;
    }

    @Override
    public void execute() {
        winch.setSpeed(SmartDashboard.getNumber("winch motor speed", 0));
        //winch.getPosition();
        //winch.setPosition(0);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.activityState = ActivityState.IDLE;
        winch.setSpeed(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
