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
    Stage stage;


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
        stage = Stage.stage0;
    }

    @Override
    public void execute()
    { 
        if(!indexer.intake() && shooter.isInterfaced() && stage == Stage.stage0) {
            acq.collect();
        } else if(stage == Stage.stage0) {
            stage = Stage.stage1;
        }

        switch(stage) {
            case stage0:
                break;
            case stage1:
                indexer.reject();
                outdexStartTime = System.currentTimeMillis();
                stage = Stage.stage2;
                break;
            case stage2:
                if (System.currentTimeMillis() >= outdexStartTime + 200) {
                    stage = Stage.stage3;
                }
                break;
            case stage3:
                if (indexer.intake()) {
                    finished = true;
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        acq.stop();
        indexer.stop(); 
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}

enum Stage {
    stage0,
    stage1,
    stage2,
    stage3
}