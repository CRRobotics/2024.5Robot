package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;

public class AmpShot extends Command implements Constants.Field, Constants.Shooter, Constants.Indexer {
    //eventually need other subsystems
    private Shooter shooter;
    private DriveTrain driveTrain;
    private Indexer indexer;
    private boolean case1;
    private boolean case2;
    private boolean case3;
    private boolean finished;
    private long outdexStartTime;
    private long shootStartTime;

    private long startTime;

    public AmpShot(Shooter shooter, DriveTrain driveTrain, Indexer indexer) {
        this.shooter = shooter;
        this.driveTrain = driveTrain;
        this.indexer = indexer;

        //purposefully didn't add drivetrain as a requirement
        addRequirements(shooter);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        // commented this section out... because why 
        // are we using this for AmpShot... not just a setpoints.


        // if(RobotContainer.shootMode.equals("visions")){
        //     if (RobotContainer.getAlliance() == Alliance.Blue)
        //     {
        //         shootAngleSpeed = ValueFromDistance.getAngleSpeedLinearized(
        //             ValueFromDistance.getDistanceToTarget(driveTrain.getPose(), ampBlue) //TODO: make this work for either side DONE?
        //         );
        //     }
        //     if (RobotContainer.getAlliance() == Alliance.Red)
        //     {
        //         shootAngleSpeed = ValueFromDistance.getAngleSpeedLinearized(
        //             ValueFromDistance.getDistanceToTarget(driveTrain.getPose(), ampRed) //TODO: make this work for either side DONE?
        //         );
        //     }
        // }
        // else
        // added new constants 2/29/24 please update in futue
        startTime = System.currentTimeMillis();
        shooter.aim(6.13);
        shooter.setSpeed(10);
        // indexer.reject();

        case1 = false;
        case2 = false;
        case3 = false;
        finished = false;
    }

    @Override
    public void execute() {
        //maybe use an enum instead of cases
        if (shooter.getAngle() > Constants.Shooter.limeLightWarningZone || Math.abs(shooter.getAngle() - Constants.Shooter.ampSetPoint) < .08) //.08 radians is quite close but idk
        {
            if(!case1)
            {
                outdexStartTime = System.currentTimeMillis();
                case1 = true;
                indexer.reject();
            }
            else if (!case2 && System.currentTimeMillis() >= outdexStartTime + 100) {
                indexer.setSpeed(0);
                case2 = true;
            }
            else if (case2 && !case3 && Math.abs(shooter.getAngle() - Constants.Shooter.ampSetPoint) < .08) {
                indexer.setSpeed(Constants.Indexer.indexShootSpeed*4.5);
                shootStartTime = System.currentTimeMillis();
                case3 = true;
            }
            else if(case3 && System.currentTimeMillis() > shootStartTime + 1000)
            {
                finished = true;
            }
            
        
    }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        // acquisition.stopAcquisitionMotor();
        shooter.aim(interfaceAngle); // i set this to interface angle because thats where it should go next.
        shooter.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (finished)
        return true;
    //TODO: cancel when button pressed
    return false;


    }
}
