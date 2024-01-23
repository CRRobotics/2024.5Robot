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

public class DriveToRealativePoint extends Command implements Constants.Auto {
    DriveTrain driveTrain;
    Pose2d relativeTarget;
    boolean finished;


    public DriveToRealativePoint(DriveTrain driveTrain, Pose2d relativeTarget) {
        this.driveTrain = driveTrain;
        this.relativeTarget = relativeTarget;
    }


    @Override
    public void initialize() {
        this.finished = false;
        List<Translation2d> list = PathPlannerPath.bezierFromPoses(
            driveTrain.getPose(),
            driveTrain.getPose().plus(new Transform2d(relativeTarget.getTranslation(), relativeTarget.getRotation()))
        );
        PathPlannerPath path = new PathPlannerPath(list, constraints, new GoalEndState(0.20, new Rotation2d(Math.PI)));
        path.preventFlipping = true;
        Command follow = AutoBuilder.followPath(path);
        follow = follow.finallyDo(
            (boolean interrupted) -> {
                this.finished = true;
        });
        follow.schedule();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
