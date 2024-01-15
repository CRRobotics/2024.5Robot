package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPoint extends SequentialCommandGroup {
    DriveTrain driveTrain;
    Pose2d target;


    public DriveToPoint(DriveTrain driveTrain, Pose2d target) {
        this.driveTrain = driveTrain;
        this.target = target;

        PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));
        
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            target,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        addCommands(
            pathfindingCommand);
    }
}