package frc.robot.commands.drivetrain;

import java.util.ArrayList;

import com.ctre.phoenix6.controls.CoastOut;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.grabber.Grab;
import frc.robot.misc.GetGlobalCoordinates;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Grabber;
import frc.robot.util.Constants;
import frc.robot.util.NetworkTableWrapper;

public class DriveToRing extends Command{
    private DriveTrain driveTrain;
    private Grabber grabber;
    private boolean isFinished = false;

    public DriveToRing(DriveTrain driveTrain, Grabber grabber) {
        this.driveTrain = driveTrain;
    }

        @Override
        public void initialize(){
            double[] pieceData = {60.0, 60.0, 0, 0};

            //pieceData = NetworkTableWrapper.getArray("","llpython");

            Translation2d closestPiece = new Translation2d(pieceData[1] * 0.0254, (pieceData[0] - 9) * -0.0254);

            GetGlobalCoordinates globalCoord = new GetGlobalCoordinates(driveTrain, closestPiece);

            Transform2d translation = new Transform2d(closestPiece.getX(), closestPiece.getY(), new Rotation2d(globalCoord.targetToRobotAngle * -1));
            DriveToRelative drive = new DriveToRelative(driveTrain, translation, true);
            drive.schedule();
            if (drive.isFinished()){
                Grab grab = new Grab(grabber);
                grab.schedule();
            }
            this.isFinished = true;
        }

        @Override
        public boolean isFinished(){
            return isFinished;
        }
}

