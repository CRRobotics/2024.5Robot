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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToRealativePoint extends Command{
    DriveTrain driveTrain;


    public DriveToRealativePoint(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }


    @Override
    public void initialize() {
        PathConstraints constraints = new PathConstraints(
        0.25, 1.0,
            Units.degreesToRadians(90), Units.degreesToRadians(180));
        List<Translation2d> list = PathPlannerPath.bezierFromPoses(driveTrain.getPose(),
        driveTrain.getPose().plus(new Transform2d(1, 0, new Rotation2d())));
        PathPlannerPath path = new PathPlannerPath(list, constraints, new GoalEndState(0,new Rotation2d(Math.PI)));
        path.preventFlipping =true;
        Command follow  = AutoBuilder.followPath(path);
        follow.schedule();
    }
}
