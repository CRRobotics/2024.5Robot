package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveToPoint;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.drivetrain.TurnToSpeaker;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.DistanceXY;

public class DriveAdjustShoot extends SequentialCommandGroup {
    
    public DriveAdjustShoot (DriveTrain drivetrain, Shooter shooter, Indexer indexer)
    {
        addCommands(
            // new DriveToPoint(drivetrain, target),
            // new TurnToSpeaker(drivetrain),
            new SpeakerShot(shooter, indexer, drivetrain)
        );
    }
}
