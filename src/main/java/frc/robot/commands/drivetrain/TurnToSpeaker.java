package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;

public class TurnToSpeaker extends Command implements Constants.Field {
    DriveTrain driveTrain;
    public TurnToSpeaker(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        double angle = Math.atan2(speakerBlue.getY() - driveTrain.getPose().getY(), speakerBlue.getX() - driveTrain.getPose().getX());
        Command command = new TurnToAngle(driveTrain, new Rotation2d(angle));
        SmartDashboard.putNumber("target rotation angle/angle", angle * 180 / Math.PI);
        SmartDashboard.putNumber("target rotation angle/y", speakerBlue.getY() - driveTrain.getPose().getY());
        SmartDashboard.putNumber("target rotation angle/x", speakerBlue.getX() - driveTrain.getPose().getY());
        SmartDashboard.putNumber("target rotation angle/angle rads", angle);
        command.schedule();
    }

}