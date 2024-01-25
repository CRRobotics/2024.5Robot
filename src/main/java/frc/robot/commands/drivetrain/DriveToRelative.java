package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Tells the drive train to use a path to a realative point on the field
 */
public class DriveToRelative extends Command {
    DriveTrain driveTrain;
    Pose2d translation;
    boolean finished;
    Command followCommand;

    public DriveToRelative(DriveTrain driveTrain, Pose2d translation) {
        this.driveTrain = driveTrain;
        this.translation = translation;
    }


    @Override
    public void initialize() {
        this.finished = false;
        List<Translation2d> list = PathPlannerPath.bezierFromPoses(
            driveTrain.getPose(),
            new Pose2d(
                driveTrain.getPose().getX() + translation.getX(),
                driveTrain.getPose().getY() + translation.getY(),
                driveTrain.getPose().getRotation().plus(translation.getRotation())
            )
        );
        PathPlannerPath path = new PathPlannerPath(list, Constants.Drive.constraints, new GoalEndState(0.20, new Rotation2d(Math.PI)));
        path.preventFlipping = true;
        followCommand = AutoBuilder.followPath(path);
        followCommand = followCommand.finallyDo(
            (boolean interrupted) -> {
                this.finished = true;
        });
        followCommand.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            followCommand.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
