package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;

public class DriveToAmp extends DriveToPoint implements Constants.Field {

    public DriveToAmp(DriveTrain driveTrain, Pose2d target) {
        super(driveTrain, new Pose2d(ampBlue, new Rotation2d(Math.PI / 2)));
    }
    
}
