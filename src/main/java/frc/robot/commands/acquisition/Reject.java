package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ActivityState;
import frc.robot.RobotContainer.ControlState;
import frc.robot.subsystems.Indexer;

/**
 * Spins the Acquisition in such a way that the game piece leaves the possession of the robot
 */
public class Reject extends Command {
    Intake acq;
    Indexer indexer;
    Shooter shooter;

    public Reject(Intake acq, Indexer indexer, Shooter shooter)
    {
        this.acq = acq;
        this.shooter = shooter;
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        RobotContainer.activityState = ActivityState.COLLECTING;
        shooter.aim(Constants.Shooter.rejectInterfaceAngle);
    }

    @Override
    public void execute()
    {
        if (shooter.isInterfaced()){
            acq.reject();
            indexer.reject();
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.activityState = ActivityState.IDLE;
        acq.stop();
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}