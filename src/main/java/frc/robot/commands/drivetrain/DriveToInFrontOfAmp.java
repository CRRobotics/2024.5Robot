package frc.robot.commands.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ActivityState;
import frc.robot.RobotContainer.ControlState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;

/**
 * Tells the drive train to use a path to drive to an absolute point on the field
 */
public class DriveToInFrontOfAmp extends Command {
    private DriveTrain driveTrain;
    private boolean finished;
    private Command pathfindingCommand;

    public DriveToInFrontOfAmp(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        Pose2d target = new Pose2d(
            RobotContainer.getAlliance().equals(Alliance.Blue)
                ? Constants.Field.ampBlue.minus(new Translation2d(0, 1))
                : Constants.Field.ampRed.minus(new Translation2d(0, 1)),
            new Rotation2d(-Math.PI / 2));
        SmartDashboard.putString("target amp", target.toString());
        // Pose2d target = new Pose2d(Constants.Field.ampBlue.minus(new Translation2d(0, 1)), new Rotation2d(-Math.PI/2));
        
        System.out.println("drive to pointing");
        // driveTrain.updateObstacles();
        pathfindingCommand = AutoBuilder.pathfindToPose(
            target,
            Constants.DriveTrain.constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        this.finished = false;
        pathfindingCommand = pathfindingCommand.finallyDo(
            (boolean interrupted) -> {
                this.finished = true;
        });
        pathfindingCommand.schedule();
        //DO NOT DELETE THIS UNDER ANY CIRCUMSTANCES
        SmartDashboard.putBoolean("pathpaht", pathfindingCommand.isScheduled());
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("activstate", Math.random());
        RobotContainer.activityState = ActivityState.IDLE;
        if (!DriverStation.isAutonomous()) RobotContainer.controlState = ControlState.MANUAL;
        pathfindingCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return finished == true;
        // return false;
    }
}