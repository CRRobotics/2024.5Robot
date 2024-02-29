package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Indexer;

/**
 * Spins the Acquisition in such a way that the game piece leaves the possession of the robot
 */
public class Reject extends Command {
    Acquisition acq;
    Indexer indexer;

    public Reject(Acquisition acq, Indexer indexer)
    {
        this.acq = acq;
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute()
    {
        acq.reject();
        indexer.reject();
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