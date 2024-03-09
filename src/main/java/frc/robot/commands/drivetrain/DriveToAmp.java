package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;

public class DriveToAmp extends DriveToPoint implements Constants.Field {

    public DriveToAmp(DriveTrain driveTrain) {
        super(driveTrain,
            new Pose2d(RobotContainer.getAlliance().equals(Alliance.Blue) ? ampBlue : ampRed, new Rotation2d(Math.PI / 2)));
    }
    
}
