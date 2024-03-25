package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.shooter.ShootAtAngleSpeed;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AngleSpeed;
import frc.robot.util.Constants;

public class ManualControl extends Command implements Constants.DriveTrain {
    private DriveTrain driveTrain;
    private Shooter shooter;
    private Indexer indexer;
    private XboxController controller = new XboxController(0);

    public ManualControl(DriveTrain driveTrain, Shooter shooter, Indexer indexer) {
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(driveTrain, shooter, indexer);
    }

    @Override
    public void initialize() {
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
        new JoystickButton(controller, XboxController.Button.kA.value).whileTrue(new ShootAtAngleSpeed(shooter, indexer, driveTrain, new AngleSpeed(shooter.getAngle(), 160)));
    }
}
