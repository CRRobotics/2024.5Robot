// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.DDRDrive;
import frc.robot.commands.drivetrain.DriveFast;
import frc.robot.commands.drivetrain.DriveSlow;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;
import frc.robot.util.DriveStates;

public class RobotContainer {
  private final DriveTrain driveTrain;
  public static DriveStates driveStates;
  private final XboxController driver;
  public static SendableChooser<String> inputMode;
  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    driveTrain = new DriveTrain();
    driveStates = DriveStates.normal;
    driver = new XboxController(Constants.Controller.driveControllerPort);
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    inputMode = new SendableChooser<>();
    addInputModes();

    driveTrain.setDefaultCommand(new JoystickDrive(driveTrain));
  }

  private void configureBindings() {
    new JoystickButton(driver, 6).whileTrue(new DriveSlow());
    new JoystickButton(driver, 5).whileTrue(new DriveFast());
  }

  private static void addInputModes() {
    inputMode.setDefaultOption("controller", "controller");
    inputMode.addOption("ddr", "ddr");
    SmartDashboard.putData("Input Mode", inputMode);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
