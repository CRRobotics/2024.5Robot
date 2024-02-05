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
    private boolean isFinished = false;
    private Command drive;

    public DriveToRing(DriveTrain driveTrain, Grabber grabber) {
        this.driveTrain = driveTrain;
        SmartDashboard.putNumber("piece/a", 0);
        SmartDashboard.putNumber("piece/b", 0);
    }

        @Override
        public void initialize(){

            // double[] pieceData = NetworkTableWrapper.getArray("limelight","llpython");
            // if (pieceData.length == 0){
            //     System.out.println("Empty Array");
            // }
            // System.out.println(pieceData[0] + ", " + pieceData[1]);
            Double[] pieceData = {SmartDashboard.getNumber("piece/a", 0), SmartDashboard.getNumber("piece/b", 0)};

            Translation2d closestPiece = new Translation2d(
                Math.sqrt(Math.pow(pieceData[1] * 0.0254, 2) - Math.pow(pieceData[0] * 0.0254, 2)),
                (pieceData[0]) * 0.0254
            );

            drive = new DriveToRelative(driveTrain, closestPiece);
            // drive = drive.finallyDo(
            //     (boolean interrupted) -> {
            //         isFinished = true;
            //     });
            drive.schedule();
        }

        @Override
        public boolean isFinished(){
            return isFinished;
        }

        @Override
        public void end(boolean interrupted) {
            System.out.println("cancelling DriveToRing");
            drive.cancel();
        }
}

