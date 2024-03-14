package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AngleSpeed;
import frc.robot.util.Constants;
import frc.robot.util.ValueFromDistance;

public class SpeakerShot extends Command implements Constants.Field, Constants.Shooter, Constants.Indexer {
    //eventually need other subsystems
    private Shooter shooter;
    private Indexer indexer;
    private DriveTrain driveTrain;
    private AngleSpeed shootAngleSpeed;
    private long outdexStartTime;
    private long indexStartTime;
    // private long shootStartTime;
    // private boolean case1;
    // private boolean case2;
    // private boolean case3;
    private boolean finished;
    private enum ShootingProgress {ALIGN, OUTDEX, INDEX}
    private ShootingProgress shootingProgress;

    public SpeakerShot(Shooter shooter, Indexer indexer, DriveTrain driveTrain) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.driveTrain = driveTrain;

        //purposefully didn't add drivetrain as a requirement
        addRequirements(shooter);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        // case1 = false;
        // case2 = false;
        // case3 = false;
        finished = false;
        // remove this line later
        // shootAngleSpeed = new AngleSpeed(SmartDashboard.getNumber("pivot setpoint", 4.3), SmartDashboard.getNumber("velocity setpoint", 0));
        Translation2d speakerPos = RobotContainer.getAlliance().equals(Alliance.Blue) ? speakerBlue : speakerRed;
        shootAngleSpeed = ValueFromDistance.getAngleSpeedLinearized(speakerPos.getDistance(driveTrain.getPose().getTranslation()));
        // shootAngleSpeed = ValueFromDistance.getAngleSpeedLinearized(1.21);
        SmartDashboard.putNumber("anglespeedaim", shootAngleSpeed.getAngle());
        SmartDashboard.putNumber("anglespeedspeed", shootAngleSpeed.getSpeed());
        shooter.aim(shootAngleSpeed.getAngle());
        shooter.setSpeed(shootAngleSpeed.getSpeed());
            
        
        shootingProgress = ShootingProgress.ALIGN;
    }
    
    // @Override
    // public void execute() {
    //     /**
    //      * Be careful about the stringent requirements on the first if statement, 
    //      * if for some reason this is true initially but then becomes false you could get undesirable behavior such as the indexer continually spitting out the note.
    //      * The shooting wheels should be spun down at the end of shooting to conserve battery and not have them run when it is not needed, 
    //      * the controller can be in charge of making sure they are up to speed in advance of shooting.
    //      * The final else if will likely not be reached since the one before it will stay true since case2 is still true and the pivot angle should remain within .08 of the set angle
    //      */
    //     if (shooter.getAngle() > Constants.Shooter.limeLightWarningZone || Math.abs(shooter.getAngle() - shootAngleSpeed.getAngle()) < .08) //.08 radians is quite close but idk
    //     {
    //         if(!case1)
    //         {
    //             outdexStartTime = System.currentTimeMillis();
    //             case1 = true;
    //             indexer.reject();
    //         }
    //         else if (!case2 && System.currentTimeMillis() >= outdexStartTime + reverseTime) {
    //             indexer.setSpeed(0);
    //             case2 = true;
    //         }
    //         else if (case2 && Math.abs(shooter.getAngle() - shootAngleSpeed.getAngle()) < .08) {
    //             indexer.setSpeed(Constants.Indexer.indexShootSpeed*4.5);
    //             shootStartTime = System.currentTimeMillis();
    //             case3 = true;
    //         }  
    //         else if(case3 && System.currentTimeMillis() > shootStartTime + 3000)
    //         {
    //             finished = true;
    //         }
    //     }      
    //     /**
    //      * if we are either at our shooting angle or are past the limelight -> reject a little bit
    //      * if we are at the shooting angle and our motors are up to speed and we are done ejecting -> shoot
    //      *  */                
    // }
    @Override
    public void execute() {
        switch (shootingProgress) {
            case ALIGN:
                shooter.aim(shootAngleSpeed.getAngle());
                if (Math.abs(shooter.getAngle() - shootAngleSpeed.getAngle()) < .08) //.08 radians is quite close but idk
                {
                    shootingProgress = ShootingProgress.OUTDEX;
                    outdexStartTime = System.currentTimeMillis();
                }
                break;
            case OUTDEX:
                indexer.reject();
                if(System.currentTimeMillis() >= outdexStartTime + outdexTime) 
                {
                    indexer.setSpeed(indexIntakeSpeed * 3);
                    shootingProgress = ShootingProgress.INDEX;
                    indexStartTime = System.currentTimeMillis();
                }
                break;
            case INDEX:
                if (System.currentTimeMillis() > indexStartTime + shootTime) 
                {
                    if (indexer.seesRing()) {
                      shootingProgress = ShootingProgress.ALIGN;  
                    }
                    else finished = true;
                }
                break;
        }     
    }


    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        shooter.aim(interfaceAngle);
        shooter.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}