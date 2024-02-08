// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;

public class TurnToAngle extends Command implements Constants.Drive, Constants.Auto.TurnToAngle {
  private DriveTrain driveTrain;
  private double setpoint;
  private boolean clockwise;
  private Rotation2d angle;

  private PIDController controller = new PIDController(kP, kI, kD);

  /**
   * Creates relative turn to angle that uses vision tracking
   * @param driveTrain DriveTrain to use
   */
  public TurnToAngle(DriveTrain driveTrain, Rotation2d angle) {
    this.driveTrain = driveTrain;
    this.angle = angle;
    addRequirements(driveTrain);
    SmartDashboard.putNumber("turnpid/p", kP);
    SmartDashboard.putNumber("turnpid/i", kI);
    SmartDashboard.putNumber("turnpid/d", kD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clockwise = Math.signum(angle.getRadians()) > 0;
    //setpoint = clockwise ? Math.abs(angle) + driveTrain.getHeading() : driveTrain.getHeading() - Math.abs(angle);
    setpoint = angle.getRadians();

    controller.setTolerance(0.01, 0.1);
    controller.setSetpoint(setpoint);
    controller.enableContinuousInput(-Math.PI, Math.PI);

    System.out.println("Auto rotate initializing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("turnpid/setpoint", controller.getSetpoint());
    SmartDashboard.putNumber("turnpid/input", driveTrain.getHeading());
    SmartDashboard.putNumber("turnpid/output", controller.calculate(driveTrain.getHeading(), setpoint));

    controller.setPID(
      SmartDashboard.getNumber("turnpid/p", kP),
      SmartDashboard.getNumber("turnpid/i", kI),
      SmartDashboard.getNumber("turnpid/d", kD)
    );
    // if (clockwise)
      turnCommand(controller.calculate(driveTrain.getPose().getRotation().getRadians(), setpoint) + 0.08);
    // else
    //   turnCommand(-controller.calculate(driveTrain.getPose().getRotation().getRadians(), setpoint) - 0.08);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // driveTrain.setSpeedsPercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  /**
 * Command to turn the robot a certain magnitude to the right
 * @param turnValue Magnitude from 0 to 1.0
 */
  public void turnCommand(double turnValue) {
      SwerveModuleState[] swerveModuleStates = driveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          0,
          0,
          turnValue,
          Rotation2d.fromRadians( RobotContainer.getAlliance().equals(Alliance.Blue) ? -driveTrain.getHeading() : driveTrain.getHeading())
        )
      );
      driveTrain.setModuleStates(swerveModuleStates);
  }
}