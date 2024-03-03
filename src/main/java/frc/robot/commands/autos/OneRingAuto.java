package frc.robot.commands.autos;
import frc.robot.commands.acquisition.Collect;
import frc.robot.commands.drivetrain.DriveToPoint;
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

    
    public OneRingAuto(Intake acq, Indexer indexer, Shooter shooter, DriveTrain drivetrain, Pose2d ringPose, Pose2d shotPose, DistanceXY distanceXY)
    {
        addCommands(

            new SpeakerShot(shooter, drivetrain, indexer, distanceXY),
            new ParallelRaceGroup
            (
                new Collect(acq, indexer, shooter),
                new DriveToPoint(drivetrain, ringPose)
            ),
            new DriveToPoint(drivetrain, shotPose),
            new SpeakerShot(shooter, drivetrain, indexer, distanceXY)
        );
    }
    
}
