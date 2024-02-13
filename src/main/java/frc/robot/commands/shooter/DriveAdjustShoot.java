package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveToPoint;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class DriveAdjustShoot extends SequentialCommandGroup {
    
    public DriveAdjustShoot (DriveTrain drivetrain, Shooter shooter, Pose2d target, Rotation2d angle)
    {
        addCommands(new DriveToPoint(drivetrain, target),
        new TurnToAngle(drivetrain, angle),
        new SpeakerShot(shooter, drivetrain));
    }
}
