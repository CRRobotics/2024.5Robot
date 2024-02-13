package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;

/**
 * Spins the Acquisition in such a way that the game piece leaves the possession of the robot
 */
public class Reject extends Command {
    Acquisition acq;

    public Reject(Acquisition acq)
    {
        this.acq = acq;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute()
    {
        acq.reject();
    }

    @Override
    public void end(boolean interrupted) {
        acq.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}