package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.util.Constants;
import frc.robot.util.Constants.Aqusition;

/**
 * Spins the Acquisition in such a way that the game piece comes into the possesion of the robot
 */
public class FastIntake extends Command implements Constants.Acquisition {
    Acquisition acq;

    public FastIntake(Acquisition acq)
    {
        this.acq = acq;
        addRequirements(acq);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute()
    {
        acq.setSpeeds(aqIntakeSpeed, aqIntakeSpeed);
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