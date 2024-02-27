package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;

//Spins motor
public class TestShot extends Command implements Constants.Shooter, Constants.Field
{
    private Shooter shooter;

    public TestShot(Shooter shooter)
    {
        this.shooter = shooter;

    }
    
    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        shooter.setSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) 
    {
        shooter.setSpeed(0);
 
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
