package frc.robot.commands.drivetrain;

import java.util.ArrayList;

import com.ctre.phoenix6.controls.CoastOut;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
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
    private DriveToRelative drive;
    private Transform2d translation;
    private int i = 0;

    public DriveToRing(DriveTrain driveTrain, Grabber grabber) {
        this.driveTrain = driveTrain;

    }

        @Override
        public void initialize(){

            // double[] pieceData = NetworkTableWrapper.getArray("limelight","llpython");
            // if (pieceData.length == 0){
            //     System.out.println("Empty Array");
            // }
            // System.out.println(pieceData[0] + ", " + pieceData[1]);
            Double[] pieceData = {20.0, 60.0};

            Translation2d closestPiece = new Translation2d(
                Math.sqrt(Math.pow(pieceData[1] * 0.0254, 2) - Math.pow(pieceData[0] * 0.0254, 2)),
                -(pieceData[0]) * 0.0254
                );
                System.out.println("closest:" + closestPiece);
            GetGlobalCoordinates globalCoord = new GetGlobalCoordinates(driveTrain, closestPiece);

            double distance = pieceData[1] * 0.0254;
            double theta = Math.atan(closestPiece.getY() / closestPiece.getX()) + driveTrain.getPose().getRotation().getRadians();
            translation = new Transform2d(distance * Math.sin(theta), -distance * Math.cos(theta), new Rotation2d(theta));
            System.out.println(translation);
            // translation = new Transform2d(closestPiece.getX(), closestPiece.getY(), new Rotation2d(Math.atan(closestPiece.getY() / closestPiece.getX())));
            drive = new DriveToRelative(driveTrain, translation, true);
            drive.schedule();
            // Transform2d target = new Transform2d(
            //     Math.sqrt(Math.pow(pieceData[1], 2) - Math.pow(pieceData[0], 2)),
            //     Math.sqrt(pieceData[0]),
            //     new Rotation2d(Math.asin(pieceData[0] / pieceData[1]))
            // );
            // DriveToRelative pathfindingCommand = new DriveToRelative(driveTrain, target, true);
            // pathfindingCommand.schedule();
        }

        @Override
        public void execute(){
            // if(i == 50){
            //     double[] pieceData = NetworkTableWrapper.getArray("limelight","llpython");
            //     System.out.println(pieceData[0] + " " + pieceData[1]);
            //     if (pieceData.length == 0){
            //     System.out.println("Empty Array");
            //     }
            //     drive.cancel();
            //     drive = new DriveToRelative(driveTrain, translation, true);
            //     drive.schedule();
            //     if (drive.isFinished()){
            //     Grab grab = new Grab(grabber);
            //     grab.schedule();
            //     isFinished = true;
            //     }
            //     i = 0;
            // }
            // i ++;
        }

        @Override
        public boolean isFinished(){
            return isFinished;
        }
}

