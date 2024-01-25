package frc.robot.commands.drivetrain;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.DriveStates;

/**
 * Commands the drive train to use slow driving
 */
public class DriveSlow extends Command{
    DriveTrain driveTrain;
    public DriveSlow()
    {
    }
    public void initialize()
    {
        RobotContainer.driveStates = DriveStates.slow;
    }

    @Override
    public void end(boolean interrupted)
    {
        RobotContainer.driveStates = DriveStates.normal;
    }
}
