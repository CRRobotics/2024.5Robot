package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveTrain;

public class DriveToSpeaker extends DriveToPoint {
    public DriveToSpeaker(DriveTrain driveTrain, Pose2d target) {
        super(driveTrain, new Pose2d(0.512318, 5.553456, new Rotation2d(Math.PI)));
    }
}
