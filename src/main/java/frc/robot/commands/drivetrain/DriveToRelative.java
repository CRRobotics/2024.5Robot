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

/**
 * Tells the drive train to use a path to a realative point on the field
 */
public class DriveToRelative extends Command {
    DriveTrain driveTrain; 
    Transform2d transformation;
    boolean calculateRotation;
    boolean finished;
    Command followCommand;


    /**
     * This is for going to point using visions data without rotation
     * @param driveTrain
     * @param transformation
     */
    public DriveToRelative(DriveTrain driveTrain, Transform2d transformation) {
        this.driveTrain = driveTrain;
        this.transformation = transformation;
        this.calculateRotation = false;
    }
    /**
    * If point requires rotation of robot, this method will do so and then pass data to DrivetoRelative to go to that point
    * @param driveTrain
    * @param translation
    */
    public DriveToRelative(DriveTrain driveTrain, Translation2d translation) {
        this(driveTrain, new Transform2d(translation, new Rotation2d()));
        this.calculateRotation = true;
    }


    @Override
    public void initialize() {
        System.out.println("running drive to relative");
        this.finished = false;
        //distance stores the hypotoneuse  of the triangle made with the point using pythagorean theorem.  
        //This is simply distance to point in straight line.
        double distance = Math.sqrt(Math.pow(transformation.getX(), 2) + Math.pow(transformation.getY(), 2));
        //This is the angle the robot needs to travel at to reach the desired point
        double theta = -Math.atan2(transformation.getY(), transformation.getX()) + driveTrain.getPose().getRotation().getRadians();
        //This if statement check whether or not a rotation needs to be made by the robot.
        if (calculateRotation) {
            transformation = new Transform2d(transformation.getTranslation(), new Rotation2d(theta + Math.PI));
        }
        //This is just pose if the robot(position using x and y, and angle of robot)
        Pose2d initPose = new Pose2d(driveTrain.getPose().getTranslation(), transformation.getRotation());
        //TODO Ask Nathan why this is here, looks like we can use data above
        Translation2d translation = new Translation2d(distance * Math.cos(theta), distance * Math.sin(theta));
        //Makes a curve from the robot to the point and allowing the robot to face wherever we want it to
        List<Translation2d> list = PathPlannerPath.bezierFromPoses(
            initPose,
            new Pose2d(initPose.getX() + translation.getX(), initPose.getY() + translation.getY(), initPose.getRotation())
        );
        //This makes a path using the bezier curve from abvove and other constants.
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
