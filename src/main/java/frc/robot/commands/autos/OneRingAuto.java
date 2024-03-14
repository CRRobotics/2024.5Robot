package frc.robot.commands.autos;
import frc.robot.commands.acquisition.Collect;
import frc.robot.commands.drivetrain.DriveToPoint;
import frc.robot.commands.shooter.DriveAdjustShoot;
import frc.robot.commands.shooter.SpeakerShot;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.DistanceXY;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneRingAuto extends SequentialCommandGroup
{
    Intake acq;
    Indexer indexer; 
    Shooter shooter; 
    Pose2d pose; 
    DriveTrain drivetrain; 

    
    public OneRingAuto(Intake acq, Indexer indexer, Shooter shooter, DriveTrain drivetrain, Pose2d ringPose, Pose2d shotPose)
    {
        addCommands(

            new DriveAdjustShoot(drivetrain, shooter, indexer),
            new ParallelRaceGroup
            (
                new Collect(acq, indexer, shooter),
                new DriveToPoint(drivetrain, ringPose)
            ),
            new DriveAdjustShoot(drivetrain, shooter, indexer)
        );
    }
    
}
