package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;

/**
 * Spins the Acquisition in such a way that the game piece comes into the possesion of the robot
 */
public class Collect extends Command implements Constants.Shooter{
    Intake acq;
    Indexer indexer;
    Shooter shooter;
    Boolean case1;
    Boolean case2;
    Boolean case3;
    Boolean case4;
    long outdexStartTime;
    long semiShotTime;
    Boolean finished;


    public Collect(Intake acq, Indexer indexer, Shooter shooter)
    {
        this.acq = acq;
        this.indexer = indexer;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.aim(Constants.Shooter.interfaceAngle);
        case1 = false;
        case2 = false;
        case3 = false;
        case4 = false;
        finished = false;
    }

    @Override
    public void execute()
    { 
        if(!indexer.intake() && shooter.isInterfaced()) {
            acq.collect();
        }
    }

    @Override
    public void end(boolean interrupted) {
        acq.stop();
        indexer.stop(); 
    }

    @Override
    public boolean isFinished() {
        if(finished)
            return true;
        return false;
    }
}