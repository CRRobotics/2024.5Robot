package frc.robot.commands.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.acquisition.Collect;
import frc.robot.commands.drivetrain.DriveToPoint;
import frc.robot.commands.shooter.SpeakerShot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.DistanceXY;

public class TwoRingAuto extends SequentialCommandGroup{
    Intake acq;
    Indexer indexer; 
    Shooter shooter; 
    Pose2d pose; 
    DriveTrain drivetrain;
    List<Pose2d> poseList; 

    //poseList needs to be (ringPose1, shotPose1, ringPose2, shotPose2)

    public TwoRingAuto(Intake acq, Indexer indexer, Shooter shooter, DriveTrain drivetrain, List<Pose2d> poseList, DistanceXY distanceXY)
    {
        addCommands(

            new SpeakerShot(shooter, drivetrain, indexer, distanceXY),
            new ParallelRaceGroup
            (
                new Collect(acq, indexer, shooter),
                new DriveToPoint(drivetrain, poseList.get(0))
            ),
            new DriveToPoint(drivetrain, poseList.get(1)),
            new SpeakerShot(shooter, drivetrain, indexer, distanceXY),
            new ParallelRaceGroup
            (
                new Collect(acq, indexer, shooter),
                new DriveToPoint(drivetrain, poseList.get(2))
            ),
            new SpeakerShot(shooter, drivetrain, indexer, distanceXY)
        );
    }
}
