package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;

/**
 * Spins the Acquisition in such a way that the game piece comes into the possesion of the robot
 */
public class Intake extends Command {
    Acquisition acq;
    Indexer indexer;
    Shooter shooter;

    public Intake(Acquisition acq, Indexer indexer, Shooter shooter)
    {
        this.acq = acq;
        this.indexer = indexer;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.aim(Constants.Shooter.interfaceAngle);
    }

    @Override
    public void execute()
    {
        if(!indexer.intake()) {
            acq.intake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        acq.stop();
        indexer.stop(); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}