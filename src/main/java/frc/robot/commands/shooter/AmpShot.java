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
import frc.robot.util.ShooterState;
import frc.robot.util.ValueFromDistance;

public class AmpShot extends Command implements Constants.Field, Constants.Shooter, Constants.Indexer {
    //eventually need other subsystems
    private Shooter shooter;
    private DriveTrain driveTrain;
    private Indexer indexer;

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
        //are we using this for AmpShot... not just a setpoints.


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
        shooter.aim(Constants.Shooter.ampSetPoint);
        shooter.setSpeed(Constants.Shooter.ampShotSpeed);
        indexer.reject();
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() >= startTime + reverseTime) {
            indexer.stop();
        }

        if (Math.abs(shooter.getSpeed() - Constants.Shooter.ampShotSpeed) < 5) { //5 is arbitraty plaese fiure out if its okay
        //    shooter.setSpeed(shootAngleSpeed.getSpeed());
        // above line likely unecessary
            indexer.intake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        // acquisition.stopAcquisitionMotor();
        shooter.aim(interfaceAngle); // i set this to interface angle because thats where it should go next.
        andThen(new WindUp(ShooterState.maxSpeed, shooter));
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() > startTime + shootTime)
            return true;
        //TODO: cancel when button pressed
        return false;


    }
}
