package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;


public class DriveToChain extends Command implements Constants.Field {
    private boolean finished;
    private Command command;
    private class Chain {
        Translation2d chainTrans;
        Rotation2d chainRot;

        private Chain(Translation2d chainTrans, Rotation2d chainRot) {
            this.chainTrans = chainTrans;
            this.chainRot = chainRot;
        }

        private Translation2d getTrans() {return chainTrans;}

        private Pose2d getTargetPos() {
            return new Pose2d(chainTrans.plus(new Translation2d(chainRot.getCos() * offset, chainRot.getSin() * offset)), chainRot);
        }
    }

    private DriveTrain driveTrain;

    public DriveToChain(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        finished = false;
    }

    @Override
    public void initialize() {
        boolean isBlue = RobotContainer.getAlliance().equals(Alliance.Blue);
        Chain ampChain = new Chain(isBlue? ampChainBlue : ampChainRed, new Rotation2d(isBlue ? (2 * Math.PI / 3) : (Math.PI / 3)));
        Chain centerChain = new Chain(isBlue? centerChainBlue : centerChainRed, new Rotation2d(isBlue? 0 : Math.PI));
        Chain sourceChain = new Chain(isBlue? sourceChainBlue : sourceChainRed, new Rotation2d(isBlue? -(2 * Math.PI / 3) : -(Math.PI / 3)));

        Chain[] chains = new Chain[]{ampChain, centerChain, sourceChain};
        Chain targetChain = new Chain(new Translation2d(10000000.0, 10000000.0), new Rotation2d(0));
        for (Chain chain : chains) {
            if (driveTrain.getPose().getTranslation().getDistance(chain.getTrans()) < driveTrain.getPose().getTranslation().getDistance(targetChain.getTrans())) {
                targetChain = chain;
            }
        }
        command = new DriveToPoint(this.driveTrain, targetChain.getTargetPos()).finallyDo(() -> finished = true);
        command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        command.cancel();
    }


    @Override
    public boolean isFinished() {
        return finished;
    }
}