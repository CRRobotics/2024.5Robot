package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.util.Constants;
import frc.robot.util.Constants.Aqusition;

/**
 * Spins the Acquisition in such a way that the game piece comes into the possesion of the robot
 */
public class PrepShot extends Command implements Constants.Acquisition {
    Acquisition acq;
    int i;
    boolean finished;

    public PrepShot(Acquisition acq)
    {
        this.acq = acq;
        addRequirements(acq);
        i = 0;
        finished = false;
    }

    @Override
    public void initialize() {
        i = 0;
    }

    @Override
    public void execute()
    {
        //Need to find right value for position
        if (i < 20) {
            acq.setSpeeds(0, aqRejectSpeed);
            i++;
        }
        else {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        acq.stop();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}