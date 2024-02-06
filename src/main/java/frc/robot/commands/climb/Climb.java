package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Winch;
import frc.robot.util.Constants;

public class Climb extends Command implements Constants.Winch
{
    Winch winch;
    Boolean extended;

    public Climb(Winch winch) {
        this.winch = winch;
        addRequirements(winch);
        extended = false;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (winch.topLeftSwitch.isPressed() == true && winch.topRightSwitch.isPressed() == true && !extended){
            winch.setSpeed(extendSpeed);
        } else {
            extended = true;
        }
        
        if (winch.bottomLeftSwitch.isPressed() == true && winch.bottomLeftSwitch.isPressed() == true && extended){
            winch.setSpeed(extendSpeed);
        }
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