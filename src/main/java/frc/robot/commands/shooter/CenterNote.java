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

    public CenterNote(Shooter shooter, Indexer indexer) {
        addRequirements(shooter);
        addRequirements(indexer);
        this.shooter = shooter;
        this.indexer = indexer;
        state = State.RETRACT;
    }

    @Override
    public void initialize() {
        state = State.RETRACT;
    }

    @Override
    public void execute() {

    }

    

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean conveyNote() {
        switch (state) {
            case NULL:
                conveyStartTime = System.currentTimeMillis();
                state = State.RETRACT;
            case RETRACT:
                if(System.currentTimeMillis() < conveyStartTime + outdexTime) {
                    indexer.reject();
                } else {
                    state = State.THROW;
                }
                break;
            case THROW:
                if(System.currentTimeMillis() < conveyStartTime + outdexTime + conveyTime) {
                    indexer.intake();
                } else {
                    state = State.GRAB;
                }
                break;
        }
    }
}
