package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.acquisition.Collect;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.NetworkTableWrapper;


public class DriveToRing extends Command{
    private DriveTrain driveTrain;
    Intake acq;
    Indexer indexer;
    Shooter shooter;
    private boolean isFinished = false;
    private Command drive;
    private Command collect;
    private int i = 0;

    /**
     * Drives robot to ring using lime light sensor
     * @param driveTrain
     */
    public DriveToRing(DriveTrain driveTrain, Intake acq, Indexer indexer, Shooter shooter) {
        this.driveTrain = driveTrain;
        this.acq = acq;
        this.indexer = indexer;
        this.shooter = shooter;
        SmartDashboard.putNumber("piece/a", 0);
        SmartDashboard.putNumber("piece/b", 0);
    }

        @Override
        public void initialize(){
            

            //Using limelight to get rings' position
            double[] pieceData = NetworkTableWrapper.getArray("limelight","llpython");
            if (pieceData.length == 0){
                System.out.println("Empty Array");
            }
            //used to troubleshoot
            System.out.println(pieceData[0] + ", " + pieceData[1]);
            // Double[] pieceData = {SmartDashboard.getNumber("piece/a", 0), SmartDashboard.getNumber("piece/b", 0)};
            //converts units from inches to centimeters
            Translation2d closestPiece = new Translation2d(
                pieceData[1] * 0.0254 + 0.13,
                (pieceData[0]) * 0.0254
            );

            drive = new DriveToRelative(driveTrain, closestPiece);
            // drive = drive.finallyDo(
            //     (boolean interrupted) -> {
            //         isFinished = true;
            //     });
            collect = new Collect(acq, indexer, shooter);
        }

        @Override
        //method recaluates path while path is running
        public void execute() {
            i++;
            double[] pieceData = NetworkTableWrapper.getArray("limelight","llpython");
            if (pieceData.length == 0){
                System.out.println("Empty Array");
            }
            if (i % 40 == 0) {
                System.out.println(pieceData[0] + ", " + pieceData[1]);
                // Double[] pieceData = {SmartDashboard.getNumber("piece/a", 0), SmartDashboard.getNumber("piece/b", 0)};

                Translation2d closestPiece = new Translation2d(
                    pieceData[1] * 0.0254 + 0.13,
                    (pieceData[0]) * 0.0254
                );

                drive = new DriveToRelative(driveTrain, closestPiece);
                // drive = drive.finallyDo(
                //     (boolean interrupted) -> {
                //         isFinished = true;
                //     });
                drive.schedule();
            }
            // if (pieceData[1] < 30) {
            //     new ParallelRaceGroup(
            //         new Collect(acq, indexer, shooter),
            //         new WaitCommand(1)
            //     ).schedule();
            // }
            SmartDashboard.putNumber("distance forward", pieceData[1]);
            SmartDashboard.putNumber("distance horizontal", pieceData[0]);
            collect.schedule();
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

