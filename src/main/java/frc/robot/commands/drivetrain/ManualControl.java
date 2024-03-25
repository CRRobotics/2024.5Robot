package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Constants;

public class ManualControl extends Command implements Constants.DriveTrain {
    private DriveTrain driveTrain;
    private Shooter shooter;
    private XboxController controller = new XboxController(0);

    public ManualControl(DriveTrain driveTrain, Shooter shooter) {
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        addRequirements(driveTrain, shooter);
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0.0,
            0.0,
            controller.getLeftX(),
            new Rotation2d(driveTrain.getGyroAngle())
        );
        SwerveModuleState[] swerveModuleStates = driveKinematics.toSwerveModuleStates(chassisSpeeds);
        driveTrain.setModuleStates(swerveModuleStates);

        shooter.setSpeedPivot(controller.getLeftY() * 0.1);
        
        if (controller.getAButtonPressed()) {
            shooter.setSpeed(160);
        } else {
            shooter.setSpeed(0);
        }
    }
}
