// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.DDRDrive;
import frc.robot.commands.drivetrain.DriveFast;
import frc.robot.commands.drivetrain.DriveSlow;
import frc.robot.commands.drivetrain.DriveToRelative;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import frc.robot.util.Constants;
import frc.robot.util.DriveStates;

public class RobotContainer {
  private final DriveTrain driveTrain;
  public static DriveStates driveStates;
  private final XboxController driver;
  public static SendableChooser<String> inputMode;
  public static LED led;
  private final SendableChooser<Command> autoChooser;
  private static SendableChooser<String> songChooser;

  public static final String industry_baby;
  public static final String old_town_road;

  static
  {
    industry_baby = "industryBaby.chrp";
    old_town_road  = "oldTownRoad.chrp";
  }

  public RobotContainer() {

    driveTrain = new DriveTrain();
    driveStates = DriveStates.normal;
    driver = new XboxController(Constants.Controller.driveControllerPort);
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    inputMode = new SendableChooser<>();
    addInputModes();
    led = new LED(60);

    songChooser = new SendableChooser<>();
    driveTrain.setDefaultCommand(new JoystickDrive(driveTrain));
  }

  
  private void configureBindings() {
    new JoystickButton(driver, 6).whileTrue(new DriveSlow());
    new JoystickButton(driver, 5).whileTrue(new DriveFast());
    // new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new DriveToRealativePoint(driveTrain));
    new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new SequentialCommandGroup(
      new DriveToRelative(driveTrain, new Pose2d(1, 0, new Rotation2d())),
      new DriveToRelative(driveTrain, new Pose2d(1, 0, new Rotation2d()))
    ));

  }

  private static void addInputModes() {
    inputMode.setDefaultOption("controller", "controller");
    inputMode.addOption("ddr", "ddr");
    SmartDashboard.putData("Input Mode", inputMode);
  }

  static {
    songChooser.setDefaultOption("Old Town Road", old_town_road);
    songChooser.addOption("Industry Baby", industry_baby);
    SmartDashboard.putData("SongChooser", songChooser);

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

  public void resetOdometry() {
    driveTrain.resetOdometry(new Pose2d());
    driveTrain.zeroHeading();
  }
}
