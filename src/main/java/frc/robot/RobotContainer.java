// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import javax.swing.JToggleButton;

import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.DDRDrive;
import frc.robot.commands.drivetrain.DriveFast;
import frc.robot.commands.drivetrain.DriveSlow;
import frc.robot.commands.drivetrain.DriveToPoint;
import frc.robot.commands.drivetrain.DriveToRelative;
import frc.robot.commands.drivetrain.DriveToRing;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.drivetrain.TurnToSpeaker;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import frc.robot.util.Constants;
import frc.robot.util.DriveStates;
import frc.robot.util.LocalADStarAK;

public class RobotContainer implements Constants.Field {
  private final DriveTrain driveTrain;
  public static DriveStates driveStates;
  // private final Grabber grabber;
  private final XboxController driver;
  public static SendableChooser<String> inputMode;
  public static LED led;
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    driveTrain = new DriveTrain();
    // grabber = new Grabber();
    driveStates = DriveStates.normal;
    driver = new XboxController(Constants.Controller.driveControllerPort);

    // NamedCommands.registerCommand("grab", new Grab(grabber));

    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    inputMode = new SendableChooser<>();
    addInputModes();
    led = new LED(60);

    driveTrain.setDefaultCommand(new JoystickDrive(driveTrain));


  }

  
  private void configureBindings() {
    new JoystickButton(driver, 6).whileTrue(new DriveSlow());
    new JoystickButton(driver, 5).whileTrue(new DriveFast());
    // new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(
    //   new DriveToRelative(driveTrain, new Translation2d(1, 2))
    // );
    // new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(
    //   new DriveToPoint(driveTrain, new Pose2d(-0.0381 + 1, 4.982718, new Rotation2d(Math.PI)))
    // );
    // // new JoystickButton(driver, XboxController.Button.kY.value).whileTrue(
    //     new DriveToRing(driveTrain, grabber)
    // );
  }

  private static void addInputModes() {
    inputMode.setDefaultOption("controller", "controller");
    inputMode.addOption("ddr", "ddr");
    SmartDashboard.putData("Input Mode", inputMode);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  public static SendableChooser<String> colorTable = new SendableChooser<>();
  public static SendableChooser<Integer> tickSpeedChooser = new SendableChooser<>();

    static{
        colorTable.addOption("red", "red");
        colorTable.addOption("blue", "blue");
        colorTable.addOption("rainbow", "rainbow");

        colorTable.setDefaultOption("orange", "orange");
        tickSpeedChooser.setDefaultOption("one", 2);
        tickSpeedChooser.addOption("one", 1);
        tickSpeedChooser.addOption("five", 5);
        tickSpeedChooser.addOption("ten", 10);

        SmartDashboard.putData(colorTable);
        SmartDashboard.putData(tickSpeedChooser);
    
    }
  public Command getDriveCommand() {
    Command driveCommand;
    switch (inputMode.getSelected()) {
      default:
        driveCommand = new JoystickDrive(driveTrain);
        break;
      case "controller":
        driveCommand = new JoystickDrive(driveTrain);
        break;
      case "ddr":
        driveCommand = new DDRDrive(driveTrain);
        break;
    }
    return driveCommand;
  }

  public static Alliance getAlliance() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            SmartDashboard.putString("alliance", "red");
            return Alliance.Red;
        }
        if (ally.get() == Alliance.Blue) {  
            SmartDashboard.putString("alliance", "blue");
            return Alliance.Blue;
        }
    }
    return null;
  }

  public void resetOdometry() {
    Pose2d pose = new Pose2d(0, 0, new Rotation2d(0)); //TODO: Test this
    if (getAlliance().equals(Alliance.Red)) {
      pose = new Pose2d(fieldWidth, 0, new Rotation2d(Math.PI));
    }
    driveTrain.resetOdometry(pose);
  }
}
