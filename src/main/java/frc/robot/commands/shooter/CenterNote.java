package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;

public class CenterNote extends Command implements Constants.Shooter, Constants.Indexer {
    private long conveyStartTime;
    private Shooter shooter;
    private Indexer indexer;
    private enum State {NULL, RETRACT, THROW, GRAB};
    State state;
    private boolean finished;

    public CenterNote(Shooter shooter, Indexer indexer) {
        addRequirements(shooter);
        addRequirements(indexer);
        this.shooter = shooter;
        this.indexer = indexer;
        state = State.NULL;
        finished = false;
    }

    @Override
    public void initialize() {
        conveyStartTime = System.currentTimeMillis();
        state = State.NULL;
    }

    @Override
    public void execute() {
        switch (state) {
            case NULL:
                conveyStartTime = System.currentTimeMillis();
                state = State.RETRACT;
                System.out.println("Centering note: retracting");
                break;
            case RETRACT:
                if(System.currentTimeMillis() < conveyStartTime + outdexTime) {
                    indexer.reject();
                } else {
                    state = State.THROW;
                    System.out.println("Centering note: throwing");
                }
                break;
            case THROW:
                if(System.currentTimeMillis() < conveyStartTime + outdexTime + conveyTime) {
                    indexer.setSpeed(indexIntakeSpeed * 3);
                } else {
                    state = State.GRAB;
                    System.out.println("Centering note: grabbing");
                }
                break;
            case GRAB:
                if (indexer.seesRing()) {
                    indexer.intake();
                } else {finished = true;}
        }
    }

    

    @Override
    public void end(boolean interrupted) {
        System.out.println("CenterNote ended");
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
