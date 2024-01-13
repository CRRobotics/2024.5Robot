package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;

public class TuneRotation extends SequentialCommandGroup {
    public TuneRotation(DriveTrain driveTrain) {
        SmartDashboard.putString("translation", driveTrain.getPose().getTranslation().toString());
        SmartDashboard.putString("holonomic rotation", driveTrain.getPose().getRotation().toString());
        SmartDashboard.putString("drivetrain heading", String.valueOf(driveTrain.getHeading()));
        Rotation2d initialHeading = Rotation2d.fromDegrees(driveTrain.getHeading());
        Translation2d targetTranslation = driveTrain.getPose().getTranslation().plus(new Translation2d(1, 0));
        Rotation2d targetHeading = new Rotation2d(0);
        Rotation2d targetHolonomicRotation = Rotation2d.fromDegrees(driveTrain.getHeading() - 90);
        SmartDashboard.putString("target", targetTranslation.toString());
        SmartDashboard.putString("target heading", targetHeading.toString());
        SmartDashboard.putString("target holonomic rotation", targetHolonomicRotation.toString());
/* 
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(0.25, 0.25),
            new PathPoint(driveTrain.getPose().getTranslation(), new Rotation2d(0), initialHeading),
            new PathPoint(targetTranslation, targetHeading,  targetHolonomicRotation)); */

        ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();
        pathPoints.add(new PathPoint(driveTrain.getPose().getTranslation()));
        pathPoints.add(new PathPoint(targetTranslation, new RotationTarget(driveTrain.getHeading(), targetHolonomicRotation)));

        PathPlannerPath path = PathPlannerPath.fromPathPoints(pathPoints, Constants.Auto.constraints, new GoalEndState(0.0, targetHolonomicRotation));

        // Assuming driveTrain.getField().getObject("traj") returns an instance of PathPlannerTrajectory

        String[] states = new String[4];
        for (int i = 0; i < driveTrain.getModuleStates().length; i++) {
            states[i] = driveTrain.getModuleStates().toString();
        }
        
        System.out.println(path.toString());
        SmartDashboard.putStringArray("module states", states);
        addCommands(
            new InstantCommand(() -> {driveTrain.resetOdometry(path.getStartingDifferentialPose());}),
            driveTrain.followPathCommand(path,
            false)
        );
    }
}
