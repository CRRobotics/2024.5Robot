package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.misc.GetGlobalCoordinates;

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

public class DriveToRelative extends Command {
    DriveTrain driveTrain;
    Transform2d transformation;
    boolean calculateRotation;
    boolean finished;
    Command followCommand;

    public DriveToRelative(DriveTrain driveTrain, Transform2d transformation) {
        this.driveTrain = driveTrain;
        this.transformation = transformation;
        this.calculateRotation = false;
    }

    public DriveToRelative(DriveTrain driveTrain, Translation2d translation) {
        this(driveTrain, new Transform2d(translation, new Rotation2d()));
        this.calculateRotation = true;
    }


    @Override
    public void initialize() {
        this.finished = false;
        double distance = Math.sqrt(Math.pow(transformation.getX(), 2) + Math.pow(transformation.getY(), 2));
        double theta = -Math.atan(transformation.getY() / transformation.getX()) + driveTrain.getPose().getRotation().getRadians();
        if (calculateRotation) {
            transformation = new Transform2d(transformation.getTranslation(), new Rotation2d(theta));
        }

        Pose2d initPose = new Pose2d(driveTrain.getPose().getTranslation(), transformation.getRotation());
        Translation2d translation = new Translation2d(distance * Math.cos(theta), distance * Math.sin(theta));
        List<Translation2d> list = PathPlannerPath.bezierFromPoses(
            initPose,
            new Pose2d(initPose.getX() + translation.getX(), initPose.getY() + translation.getY(), initPose.getRotation())
        );

        PathPlannerPath path = new PathPlannerPath(list, Constants.Drive.constraints, new GoalEndState(0, transformation.getRotation()));
        path.preventFlipping = true;
        // followCommand = AutoBuilder.followPath(path);
        followCommand = AutoBuilder.followPath(path);
        // followCommand = followCommand.finallyDo(
        //     (boolean interrupted) -> {
        //         this.finished = true;
        // });
        followCommand.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        followCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
