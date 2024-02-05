package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber;

public class Grab extends CommandBase {
    Grabber grabber;
    XboxController controller = new XboxController(0);
    
    public Grab(Grabber grabber)
    {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        grabber.setSpeed(0.6);
    }

    @Override
    public void execute()
    {
        grabber.setSpeed(0.6);
    }

    @Override
    public void end(boolean interrupted) {
        grabber.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}