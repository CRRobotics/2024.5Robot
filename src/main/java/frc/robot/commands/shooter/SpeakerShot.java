package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AngleSpeed;
import frc.robot.util.Constants;
import frc.robot.util.ValueFromDistance;

public class SpeakerShot extends Command implements Constants.Field, Constants.Shooter {
    //eventually need other subsystems
    private Shooter shooter;
    private DriveTrain driveTrain;

    private long startTime;
    private AngleSpeed shootAngleSpeed;

    public SpeakerShot(Shooter shooter, DriveTrain driveTrain) {
        this.shooter = shooter;
        this.driveTrain = driveTrain;

        //purposefully didn't add drivetrain as a requirement
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shootAngleSpeed = ValueFromDistance.getAngleSpeedLinearized(
            ValueFromDistance.getDistanceToTarget(driveTrain.getPose(), speakerBlueTarget) //TODO: make this work for either side
        );
        startTime = System.currentTimeMillis();
        shooter.aim(shootAngleSpeed.getAngle());
        shooter.setSpeed(reverseIndexSpeed);
        // indexer.setIndexMotor(reverseIndexSpeed);
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() >= startTime + reverseTime) {
            // indexer.setIndexMotor(0);
            shooter.setSpeed(shootAngleSpeed.getSpeed());
        }

        if (System.currentTimeMillis() >= startTime + spinUpTime) {
            shooter.setSpeed(shootAngleSpeed.getSpeed());
            // indexer.setIndexMotor(Constants.IndexerConstants.indexMotorSpeed);
            // acquisition.spinAcquisition(Constants.AcquisitionConstants.acquisitionSpeedSlow);
        }


    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
        // indexer.setIndexMotor(0);
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