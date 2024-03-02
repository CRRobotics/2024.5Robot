package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AngleSpeed;
import frc.robot.util.Constants;
import frc.robot.util.DistanceXY;
import frc.robot.util.ShooterState;
import frc.robot.util.ValueFromDistance;

public class SpeakerShot extends Command implements Constants.Field, Constants.Shooter, Constants.Indexer {
    //eventually need other subsystems
    private Shooter shooter;
    private DriveTrain driveTrain;
    private Indexer indexer;
    private DistanceXY distanceXY;
    private long startTime;
    private boolean outdex;
    private AngleSpeed shootAngleSpeed;
    private long outdexStartTime;

    public SpeakerShot(Shooter shooter, DriveTrain driveTrain, Indexer indexer, DistanceXY distanceXY) {
        this.shooter = shooter;
        this.driveTrain = driveTrain;
        this.distanceXY = distanceXY;
        shootAngleSpeed = ValueFromDistance.getAngleSpeedLinearized(distanceXY.getDistanceToSpeaker());
        this.indexer = indexer;
        outdex = false;

        //purposefully didn't add drivetrain as a requirement
        addRequirements(shooter);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        // if (RobotContainer.getAlliance() == Alliance.Blue)
        // {
        //     shootAngleSpeed = ValueFromDistance.getAngleSpeedLinearized(
        //         ValueFromDistance.getDistanceToTarget(driveTrain.getPose(), speakerBlue) //TODO: make this work for either side DONE?
        //     );
        // }
        // if (RobotContainer.getAlliance() == Alliance.Red)
        // {
        //     shootAngleSpeed = ValueFromDistance.getAngleSpeedLinearized(
        //         ValueFromDistance.getDistanceToTarget(driveTrain.getPose(), speakerRed) //TODO: make this work for either side DONE?
        //     );
        // }
        startTime = System.currentTimeMillis();
        // shooter.aim(shootAngleSpeed.getAngle());
        // shooter.setSpeed(shootAngleSpeed.getSpeed());
        if(RobotContainer.inputMode.getSelected().equals("test")){
            shooter.aim(SmartDashboard.getNumber("pivot setpoint", 4.3));
            shooter.setSpeed(SmartDashboard.getNumber("velocity setpoint", 0));
        }
        else
        {
            shooter.aim(shootAngleSpeed.getAngle());
            shooter.setSpeed(shootAngleSpeed.getSpeed());
            
        }
    }

    @Override
    public void execute() {
        if(RobotContainer.inputMode.getSelected().equals("test"))
        {
            if (Math.abs(shooter.getSpeed() - SmartDashboard.getNumber("velocity setpoint", 0)) < 10) 
            {
            
                if (shooter.getAngle() > Constants.Shooter.limeLightWarningZone || shooter.getAngle() == SmartDashboard.getNumber("pivot setpoint", 4.3)) {
                    if(!outdex)
                    {
                        outdexStartTime = System.currentTimeMillis();
                        outdex = true;
                    }
                    if (System.currentTimeMillis() <= outdexStartTime + reverseTime) {
                        indexer.reject();
                    }
                    else
                    {
                        indexer.setSpeed(Constants.Indexer.indexShootSpeed);
                    }
                }
            }    
        
        }
        else
        {
            if (Math.abs(shooter.getSpeed() - shootAngleSpeed.getSpeed()) < 10) 
            {
            
                if (shooter.getAngle() > Constants.Shooter.limeLightWarningZone || shooter.getAngle() == shootAngleSpeed.getAngle()) {
                    if(!outdex)
                    {
                        outdexStartTime = System.currentTimeMillis();
                        outdex = true;
                    }
                    if (System.currentTimeMillis() <= outdexStartTime + reverseTime) {
                        indexer.reject();
                    }
                    else
                    {
                        indexer.setSpeed(Constants.Indexer.indexShootSpeed);
                    }
                }
                
            }

        }
        
        
        /**
         * if we are either at our shooting angle or are past the limelight -> reject a little bit
         * if we are at the shooting angle and our motors are up to speed and we are done ejecting -> shoot
         *  */        
        
        
        
       
      
        
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
        indexer.stop();
        shooter.aim(interfaceAngle);
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() > startTime + shootTime)
            return true;
        //TODO: cancel when button pressed
        return false;


}
}