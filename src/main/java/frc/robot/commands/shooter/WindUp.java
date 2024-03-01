package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;
import frc.robot.util.ShooterState;

public class WindUp extends Command {
    
    private Shooter shooter;
    public WindUp(ShooterState shooterState, Shooter shooter)
    {
        RobotContainer.shooterState = shooterState;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        switch (RobotContainer.shooterState) {
            case maxSpeed: 
                shooter.setSpeed(Constants.Shooter.shooterDefaultMaxSpeed);
                break;
            case notSpinning:
                shooter.setSpeed(0);
                break;
            case ampSpeed:
                shooter.setSpeed(Constants.Shooter.ampShotSpeed);
                break;
        }
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
