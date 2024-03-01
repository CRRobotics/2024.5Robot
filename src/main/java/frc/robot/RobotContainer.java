// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.TreeMap;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.acquisition.Intake;
import frc.robot.commands.acquisition.Reject;
import frc.robot.commands.drivetrain.DDRDrive;
import frc.robot.commands.drivetrain.DriveFast;
import frc.robot.commands.drivetrain.DriveSlow;
import frc.robot.commands.drivetrain.DriveToRing;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.shooter.SpeakerShot;
import frc.robot.commands.shooter.TestPivot;
import frc.robot.commands.shooter.TestShot;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AngleSpeed;
import frc.robot.util.Constants;
import frc.robot.util.DriveStates;

public class RobotContainer {
  private final DriveTrain driveTrain;
  private final Acquisition acq;
  private final Indexer indexer;
  public static DriveStates driveStates;
  private final XboxController driver;
  public static SendableChooser<String> inputMode;
  public static LED led;
  private final SendableChooser<Command> autoChooser;
  private final Shooter shooter;

  public RobotContainer() {
    driveTrain = new DriveTrain();
    acq = new Acquisition();
    indexer = new Indexer();
    shooter = new Shooter();
    driveStates = DriveStates.normal;
    driver = new XboxController(Constants.Controller.driveControllerPort);
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    inputMode = new SendableChooser<>();
    addInputModes();
    led = new LED(60);

    driveTrain.setDefaultCommand(new JoystickDrive(driveTrain));
  }
  public static TreeMap<Double, AngleSpeed> treeMap;

  static
  {
    treeMap = new TreeMap<Double, AngleSpeed>();
    //treeMap.put(null, null);
    
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
  
  private void configureBindings() {
    new JoystickButton(driver, 6).whileTrue(new DriveSlow());
    new JoystickButton(driver, 5).whileTrue(new DriveFast());
    // new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new DriveToRealativePoint(driveTrain));
    // new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new SequentialCommandGroup(
    //  new DriveToRelative(driveTrain, new Transform2d(1, 0, new Rotation2d())),
    //  new DriveToRelative(driveTrain, new Transform2d(1, 0, new Rotation2d()))
    // ));
    //new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(new SequentialCommandGroup(
    //  new DriveToPoint(driveTrain, new Pose2d(3, 0, new Rotation2d()))
    //));
    new JoystickButton(driver, XboxController.Button.kX.value).whileTrue(new Intake(acq, indexer, shooter)); //Assign Button
    new JoystickButton(driver, XboxController.Button.kY.value).whileTrue(new Reject(acq, indexer));
    new JoystickButton(driver, XboxController.Button.kA.value).onTrue(new TestPivot(shooter));
    // new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(new SpeakerShot(shooter, driveTrain, indexer));
    new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(new DriveToRing(driveTrain));

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

  static {
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

  public void resetOdometry() {
    driveTrain.resetOdometry(new Pose2d());
    driveTrain.zeroHeading();
  }
}