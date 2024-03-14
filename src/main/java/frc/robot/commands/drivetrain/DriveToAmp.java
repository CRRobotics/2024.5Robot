package frc.robot.commands.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;

/**
 * Tells the drive train to use a path to drive to an absolute point on the field
 */
public class DriveToAmp extends Command {
    DriveTrain driveTrain;
    boolean finished;
    Command pathfindingCommand;

    public DriveToAmp(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        Pose2d target = new Pose2d(RobotContainer.getAlliance().equals(Alliance.Blue) ? Constants.Field.ampBlue : Constants.Field.ampRed, new Rotation2d(Math.PI / 2));
        System.out.println("drive to pointing");
        driveTrain.updateObstacles();
        pathfindingCommand = AutoBuilder.pathfindToPose(
            target,
            Constants.Auto.constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        this.finished = false;
        // pathfindingCommand = pathfindingCommand.finallyDo(
        //     (boolean interrupted) -> {
        //         this.finished = true;
        // });
        pathfindingCommand.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        pathfindingCommand.end(true);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}