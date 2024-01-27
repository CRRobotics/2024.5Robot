package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AngleSpeed;
import frc.robot.util.Constants;
import frc.robot.util.ValueFromDistance;

public class SpeakerShot extends Command implements Constants.Field {
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
        if(!Robot.lockedOn())
            end(true);
            
        shootAngleSpeed = ValueFromDistance.getAngleSpeedLinearized(ValueFromDistance.getDistanceToTarget(driveTrain.getPose(), ));
        shooter.setCoast();
        startTime = System.currentTimeMillis();
        shooter.setActuator(shootAngleSpeed.getAngle());
        shooter.setSpeed(Constants.ShooterConstants.reverseIndexSpeed);
        indexer.setIndexMotor(-Constants.IndexerConstants.indexMotorSpeed);

        //acquisition.acquisitionNeutral();
        acquisition.acquisitionDown();
        led.redFlare();
    }

    @Override
    public void execute() {
        if(System.currentTimeMillis() >= startTime + Constants.ShooterConstants.reverseIndexWhenShootingTime)
        {
            indexer.setIndexMotor(0);
            shooter.setSpeedRPM(shootAngleSpeed.getSpeed());
        }
        if(System.currentTimeMillis() >= startTime + Constants.ShooterConstants.reverseIndexWhenShootingTime + Constants.ShooterConstants.spinUpTime)
        {
            shooter.setSpeedRPM(shootAngleSpeed.getSpeed());
            indexer.setIndexMotor(Constants.IndexerConstants.indexMotorSpeed);
            acquisition.spinAcquisition(Constants.AcquisitionConstants.acquisitionSpeedSlow);
        }


    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
        indexer.setIndexMotor(0);
        acquisition.stopAcquisitionMotor();
        shooter.setBrake();
    }

    @Override
    public boolean isFinished() {
        if(System.currentTimeMillis() > startTime + Constants.ShooterConstants.pureShootingTime)
            return true;
        if(ControllerWrapper.DriverRightTrigger.get() || ControllerWrapper.ControllerRightTrigger.get())
        {
            return true;
        }
        return false;


}
}