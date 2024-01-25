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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Tells the drive train to use a path to drive to an absolute point on the field
 */
public class DriveToPoint extends Command {
    DriveTrain driveTrain;
    Pose2d target;
    boolean finished;

    public DriveToPoint(DriveTrain driveTrain, Pose2d target) {
        this.driveTrain = driveTrain;
        this.target = target;
    }

        @Override
        public void initialize() {
        driveTrain.updateObstacles();
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            target,
            Constants.Drive.constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        this.finished = false;
        pathfindingCommand = pathfindingCommand.finallyDo(
            (boolean interrupted) -> {
                this.finished = true;
        });
        pathfindingCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}