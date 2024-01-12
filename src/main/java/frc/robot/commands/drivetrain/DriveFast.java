package frc.robot.commands.drivetrain;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.DriveStates;
public class DriveFast extends CommandBase{
    DriveTrain driveTrain;
    public DriveFast(){}
    public void initialize()
    {
        RobotContainer.driveStates = DriveStates.speeed;
    }

    @Override
    public void end(boolean interrupted)
    {
        RobotContainer.driveStates = DriveStates.normal;
    }
}
