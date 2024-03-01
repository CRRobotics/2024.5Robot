package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.DriveTrain;

public class DistanceXY {
    DriveTrain driveTrain;
    Alliance alliance;
    public DistanceXY(DriveTrain driveTrain, Alliance alliance)
    {
        this.driveTrain = driveTrain;
        this.alliance = alliance;
    }

    public double getDistanceToSpeaker()
    {
        if(alliance.equals(Alliance.Red))
        {
            return Math.sqrt(
                Math.pow(Constants.Field.speakerRed.getX() - driveTrain.getPose().getX(), 2) +
                Math.pow(Constants.Field.speakerRed.getY() - driveTrain.getPose().getY(), 2)
            );
        }
        else
        {
            return Math.sqrt(
                Math.pow(Constants.Field.speakerBlue.getX() - driveTrain.getPose().getX(), 2) +
                Math.pow(Constants.Field.speakerBlue.getY() - driveTrain.getPose().getY(), 2)

            );
        }
    }
}
