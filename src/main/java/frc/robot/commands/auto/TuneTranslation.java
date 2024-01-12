package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.Constants;
import frc.robot.subsystems.DriveTrain;

public class TuneTranslation extends SequentialCommandGroup {
    public TuneTranslation(DriveTrain driveTrain) {
        addCommands(
            driveTrain.followTrajectoryCommand(PathPlanner.generatePath(
                Constants.Auto.constraints,
                new PathPoint(driveTrain.getPose().getTranslation(), driveTrain.getPose().getRotation()),
                new PathPoint(driveTrain.getPose().getTranslation().plus(new Translation2d(1, 0)), driveTrain.getPose().getRotation())
            ),
            false)
        );
    }
}
