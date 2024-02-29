package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;

public class TestPivot extends Command implements Constants.Shooter 
{
    private Shooter shooter; 

    public TestPivot(Shooter shooter)
    {
        this.shooter = shooter;
        SmartDashboard.putNumber("pivot setpoint", 5.5);
    }

    @Override
    public void initialize()
    {
    }

    //positive direction spins clockwise
    @Override
    public void execute()
    {
        shooter.testAim(SmartDashboard.getNumber("pivot setpoint", 5.5));
    }

    @Override
    public void end(boolean interrupted) 
    {
        shooter.setSpeedPivot(0);
    }
}
