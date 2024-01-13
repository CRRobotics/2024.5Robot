package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class FollowAuto extends SequentialCommandGroup {
    public FollowAuto(DriveTrain driveTrain, String auto) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(auto);
        addCommands(driveTrain.followPathCommand(path));
    }
}