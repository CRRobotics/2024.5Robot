package frc.robot.commands.auto;
import java.util.ArrayList;


import java.util.List;

import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

import frc.robot.util.Constants;

public class TuneTranslation extends SequentialCommandGroup {
    public TuneTranslation(DriveTrain driveTrain) {
        ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();

        pathPoints.add(new PathPoint(driveTrain.getPose().getTranslation(), new RotationTarget(0.0, driveTrain.getPose().getRotation())));
        pathPoints.add(new PathPoint(driveTrain.getPose().getTranslation().plus(new Translation2d(1, 0)), new RotationTarget(0.0, driveTrain.getPose().getRotation())));
        PathPlannerPath path = PathPlannerPath.fromPathPoints(pathPoints, Constants.Auto.constraints, new GoalEndState(0.0, new Rotation2d(0)));
        
        addCommands(
            driveTrain.followPathCommand(path, false)
        );
    }
}