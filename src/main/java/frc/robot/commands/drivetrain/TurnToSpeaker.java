package frc.robot.commands.drivetrain;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ActivityState;
import frc.robot.RobotContainer.ControlState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;
/**
 * 
 */
public class TurnToSpeaker extends Command implements Constants.Field {
    DriveTrain driveTrain;
    Command command;
    boolean done;


    public TurnToSpeaker(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    ControlState oldControlState;

    @Override
    public void initialize() {
        RobotContainer.activityState = ActivityState.DRIVING;
        if (!DriverStation.isAutonomous()) RobotContainer.controlState = ControlState.PATHING;
        done = false;
        Translation2d speakerPos = RobotContainer.getAlliance().equals(Alliance.Blue) ? speakerBlue : speakerRed;
        double angle = Math.atan2(speakerPos.getY() - driveTrain.getPose().getY(), speakerPos.getX() - driveTrain.getPose().getX());
        Field2d field = new Field2d();
        // field.setRobotPose(driveTrain.getPose().getX(), driveTrain.getPose().getY(), new Rotation2d(angle));
        SmartDashboard.putData(field);
        command = new TurnToAngle(driveTrain, new Rotation2d(angle));
        SmartDashboard.putNumber("target rotation angle/angle", angle * 180 / Math.PI);
        SmartDashboard.putNumber("target rotation angle/y", speakerPos.getY() - driveTrain.getPose().getY());
        SmartDashboard.putNumber("target rotation angle/x", speakerPos.getX() - driveTrain.getPose().getY());
        SmartDashboard.putNumber("target rotation angle/angle rads", angle);
        command = command.finallyDo((boolean interrupted) -> new RunCommand(() -> done = true));
        command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.activityState = ActivityState.IDLE;
        if (!DriverStation.isAutonomous()) RobotContainer.controlState = ControlState.MANUAL;
        command.cancel();
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}