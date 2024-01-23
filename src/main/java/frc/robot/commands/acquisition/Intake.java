package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Acquisition;

public class Intake extends CommandBase
{
    Acquisition acq;
    XboxController controller = new XboxController(0);

    public Intake(Acquisition acq)
    {
        this.acq = acq;
        addRequirements(acq);
    }

    @Override
    public void initialize() {
        acq.setSpeed(0.6);
    }

    @Override
    public void execute()
    {
        acq.setSpeed(0.6);
    }

    @Override
    public void end(boolean interrupted) {
        acq.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}