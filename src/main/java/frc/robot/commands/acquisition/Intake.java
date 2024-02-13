package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Indexer;

/**
 * Spins the Acquisition in such a way that the game piece comes into the possesion of the robot
 */
public class Intake extends Command {
    Acquisition acq;
    Indexer indexer;

    public Intake(Acquisition acq, Indexer indexer)
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
        acq.intake();
        //indexer.intake(); No physical indexer yet, TODO: uncomment when ready
    }

    @Override
    public void end(boolean interrupted) {
        acq.stop();
        //indexer.stop(); No physical indexer yet, TODO: uncomment when ready
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}