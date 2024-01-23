package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveTrain;

public class DriveToAmp extends DriveToPoint {

    public DriveToAmp(DriveTrain driveTrain, Pose2d target) {
        super(driveTrain, new Pose2d(1.93294, 4.867656, new Rotation2d(Math.PI / 2)));
    }
    
}
