// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.TuneRotation;
import frc.robot.commands.auto.TuneTranslation;
import frc.robot.commands.auto.FollowAuto;
// import frc.robot.commands.auto.TuneRotation;
// import frc.robot.commands.auto.TuneTranslation;
import frc.robot.commands.drivetrain.DDRDrive;
import frc.robot.commands.drivetrain.DriveFast;
import frc.robot.commands.drivetrain.DriveSlow;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import frc.robot.util.Constants;
import frc.robot.util.DriveStates;

public class RobotContainer {
  private final DriveTrain driveTrain;
  public static DriveStates driveStates;
  private final XboxController driver;
  public static SendableChooser<String> autoMode;
  public static SendableChooser<String> inputMode;
  public static LED led;


  public RobotContainer() {

    driveTrain = new DriveTrain();
    driveStates = DriveStates.normal;
    driver = new XboxController(Constants.Controller.driveControllerPort);
    configureBindings();
    autoMode = new SendableChooser<>();
    addAutoModes();
    inputMode = new SendableChooser<>();
    addInputModes();
    led = new LED(60);

    driveTrain.setDefaultCommand(new JoystickDrive(driveTrain));
  }

  private void configureBindings() {
    new JoystickButton(driver, 6).whileTrue(new DriveSlow());
    new JoystickButton(driver, 5).whileTrue(new DriveFast());
  }

  private static void addAutoModes() {
    autoMode.addOption("Tune Translation", "Tune Translation");
    autoMode.addOption("Tune Rotation", "Tune Rotation");
    autoMode.addOption("NewPath", "NewPath");
    autoMode.addOption("Short", "Short");
    SmartDashboard.putData("Auto Mode", autoMode);
  }

  private static void addInputModes() {
    inputMode.setDefaultOption("controller", "controller");
    inputMode.addOption("ddr", "ddr");
    SmartDashboard.putData("Input Mode", inputMode);
  }

  public Command getAutonomousCommand() {
    Command auto;
    switch (autoMode.getSelected()) {
      default:
        auto = new FollowAuto(driveTrain, autoMode.getSelected());
        break;
      case "Tune Translation":
        auto = new TuneTranslation(driveTrain);
        break;
      case "Tune Rotation":
        auto = new TuneRotation(driveTrain);
        break;
    }
    driveTrain.resetOdometry(new Pose2d(), new Rotation2d(0));
    return auto;
  }
  public static SendableChooser<String> colorTable = new SendableChooser<>();

    static{
        colorTable.addOption("red", "red");
        colorTable.addOption("blue", "blue");
        colorTable.setDefaultOption("orange", "orange");
        SmartDashboard.putData(colorTable);
    
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
}
