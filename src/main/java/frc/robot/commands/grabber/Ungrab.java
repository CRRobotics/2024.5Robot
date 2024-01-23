package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Grabber;

public class Ungrab extends CommandBase{
    Grabber grabber;
    XboxController controller = new XboxController(0);
    
    public Ungrab(Grabber grabber)
    {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        // grabber.setCurrentLimit(10);
    }

    @Override
    public void execute()
    {
        grabber.setSpeed(-1);
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