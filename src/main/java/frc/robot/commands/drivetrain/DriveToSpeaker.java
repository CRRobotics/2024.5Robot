package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;

public class DriveToSpeaker extends DriveToPoint implements Constants.Field {
    public DriveToSpeaker(DriveTrain driveTrain, Pose2d target) {
        super(driveTrain, new Pose2d(speakerBlue, new Rotation2d(Math.PI)));
    }
}
