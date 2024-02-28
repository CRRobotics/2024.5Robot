package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AngleSpeed;
import frc.robot.util.Constants;
import frc.robot.util.ValueFromDistance;

public class AmpShotTest extends Command implements Constants.Shooter, Constants.Indexer {
    //eventually need other subsystems
    private Shooter shooter;
    private DriveTrain driveTrain;
    private Indexer indexer;

    private long startTime;
    private AngleSpeed shootAngleSpeed;

    public AmpShotTest(Shooter shooter, Indexer indexer) {
        this.shooter = shooter;
        this.indexer = indexer;

        //purposefully didn't add drivetrain as a requirement
        addRequirements(shooter);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        shooter.aim(shootAngleSpeed.getAngle());
        shooter.setSpeed(reverseIndexSpeed);
        indexer.reject();
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() >= startTime + reverseTime) {
            indexer.stop();
            shooter.setSpeed(shootAngleSpeed.getSpeed());
        }

        if (System.currentTimeMillis() >= startTime + spinUpTime) {
            shooter.setSpeed(shootAngleSpeed.getSpeed());
            indexer.intake();
        }


    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
        indexer.stop();
        // acquisition.stopAcquisitionMotor();
        shooter.aim(restAngle);
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() > startTime + shootTime)
            return true;
        //TODO: cancel when button pressed
        return false;


    }
}
