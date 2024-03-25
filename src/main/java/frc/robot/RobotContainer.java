// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.acquisition.Collect;
import frc.robot.commands.acquisition.Reject;
import frc.robot.commands.climb.Climb;
import frc.robot.commands.climb.Extend;
import frc.robot.commands.drivetrain.DDRDrive;
import frc.robot.commands.drivetrain.DriveFast;
import frc.robot.commands.drivetrain.DriveSlow;
import frc.robot.commands.drivetrain.DriveToAmp;
import frc.robot.commands.drivetrain.DriveToInFrontOfAmp;
import frc.robot.commands.drivetrain.DriveToPoint;
import frc.robot.commands.drivetrain.DriveToRing;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.ManualControl;
import frc.robot.commands.drivetrain.TurnToSpeaker;
import frc.robot.commands.shooter.AmpShot;
import frc.robot.commands.shooter.BumbperShot;
import frc.robot.commands.shooter.CenterNote;
import frc.robot.commands.shooter.SpeakerShot;
import frc.robot.commands.shooter.WindUp;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Winch;
import frc.robot.util.Constants;
import frc.robot.util.Constants.Auto.NotePositions;
import frc.robot.util.DriveStates;
import frc.robot.util.ShooterState;

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
  // SUBSYSTEMS
  private final DriveTrain driveTrain = new DriveTrain();
  private final Intake acq = new Intake();
  private final Indexer indexer = new Indexer();
  private final Shooter shooter = new Shooter();
  private final Winch winch = new Winch();
  public static LED led = new LED(50, 2, 15); // TODO: Set LED length

  // STATES
  /** Speed modes */
  public static DriveStates driveStates;
  /** Manages the speed of the shooter */
  public static ShooterState shooterState;

  // FINAL IO
  private final XboxController driver = new XboxController(Constants.Controller.driverControllerPort);
  private final XboxController operator = new XboxController(Constants.Controller.operatorControllerPort);

  // STATIC IO
  /** Between DDR pad and controller input */
  public static SendableChooser<String> inputMode = new SendableChooser<>();
  public static SendableChooser<Pose2d> ringPositionChooser = new SendableChooser<>();
  public static SendableChooser<String> autoCommandChooser = new SendableChooser<>();
  public static SendableChooser<Pose2d> startingPos = new SendableChooser<>();

  /**
   * Constructs a new RobotContainer. This constructor is responsible for setting up the robot's subsystems and commands.
   * It initializes the LED, drive states, distance measurement, input modes, and shooter state.
   * It also sets the default command for the drive train to be JoystickDrive.
   */
  public RobotContainer() {
    driveStates = DriveStates.normal;
    
    // ROBOT CONFIGURATION
    configureSendableChoosers();
    configureBindings();
    setShooterState(ShooterState.notSpinning);
    driveTrain.setDefaultCommand(new JoystickDrive(driveTrain, driver));
  }

  private void configureSendableChoosers() {
    inputMode.setDefaultOption("controller", "controller");
    inputMode.addOption("ddr", "ddr");
    inputMode.addOption("demo", "demo");
    SmartDashboard.putData("Input Mode", inputMode);

    ringPositionChooser.addOption("LeftRed", new Pose2d(NotePositions.kNotesStartingRedWing[2], new Rotation2d(Math.PI)));
    ringPositionChooser.addOption("MiddleRed",  new Pose2d(NotePositions.kNotesStartingRedWing[1], new Rotation2d(Math.PI)));
    ringPositionChooser.addOption("RightRed",  new Pose2d(NotePositions.kNotesStartingRedWing[0], new Rotation2d(Math.PI)));
    ringPositionChooser.addOption("LeftBlue", new Pose2d(NotePositions.kNotesStartingBlueWing[2], new Rotation2d(0)));
    ringPositionChooser.addOption("MiddleBlue",  new Pose2d(NotePositions.kNotesStartingBlueWing[1], new Rotation2d(0)));
    ringPositionChooser.addOption("RightBlue",  new Pose2d(NotePositions.kNotesStartingBlueWing[0], new Rotation2d(0)));
    ringPositionChooser.addOption("Mid1", new Pose2d(8.28, 7.42, new Rotation2d(Math.PI / 4)));
    ringPositionChooser.addOption("Mid2", new Pose2d(8.28, 5.77, new Rotation2d(Math.PI / 4)));
    ringPositionChooser.addOption("Mid3", new Pose2d(8.28, 4.11, new Rotation2d(Math.PI / 4)));
    ringPositionChooser.addOption("Mid4", new Pose2d(8.28, 2.42, new Rotation2d(Math.PI / 4)));
    ringPositionChooser.addOption("Mid5", new Pose2d(8.28, 0.76, new Rotation2d(Math.PI / 4)));
    ringPositionChooser.setDefaultOption("LeftRed", new Pose2d(NotePositions.kNotesStartingRedWing[2], new Rotation2d(Math.PI)));
    SmartDashboard.putData(ringPositionChooser);

    autoCommandChooser.addOption("OneRing", "OneRing");
    autoCommandChooser.addOption("Shoot", "Shoot");
    autoCommandChooser.addOption("Nothing", "Nothing");
    autoCommandChooser.addOption("Mobility", "Mobility");
    autoCommandChooser.addOption("Mess With 2 Notes", "Mess With 2 Notes");
    autoCommandChooser.addOption("Mess With Note", "Mess With Note");
    autoCommandChooser.setDefaultOption("Nothing", "Nothing");
    SmartDashboard.putData(autoCommandChooser);

    startingPos.addOption("amp side", new Pose2d(0.68, 6.58, Rotation2d.fromDegrees(60)));
    startingPos.addOption("subwoofer", new Pose2d(1.19, 5.57, Rotation2d.fromDegrees(0)));
    startingPos.addOption("not amp side", new Pose2d(0.68, 4.57, Rotation2d.fromDegrees(-60)));
    startingPos.addOption("source side", new Pose2d(0.39, 1.98, Rotation2d.fromDegrees(0)));
    startingPos.setDefaultOption("amp side", new Pose2d(0.68, 6.58, Rotation2d.fromDegrees(60)));
    SmartDashboard.putData(startingPos);
  }

  /**
   * Binds specific commands to specific buttons and axis on the driver and operator controllers.
   */
  private void configureBindings() {
    // DRIVER BINDINGS
    new JoystickButton(driver, XboxController.Button.kRightBumper.value).whileTrue(new DriveSlow());
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value).whileTrue(new DriveFast());
    new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(new DriveToRing(driveTrain, acq, indexer, shooter));
    new JoystickButton(driver, XboxController.Button.kBack.value).whileTrue(new Climb(winch, shooter).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    new JoystickButton(driver, XboxController.Button.kStart.value).onTrue(new Extend(winch));
    new JoystickButton(driver, XboxController.Button.kLeftStick.value).onTrue(new RunCommand(() -> CommandScheduler.getInstance().cancelAll()));
    new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(
      new SequentialCommandGroup(
        new DriveToInFrontOfAmp(driveTrain),
        new DriveToAmp(driveTrain)
      )
    );
    new JoystickButton(driver, XboxController.Button.kX.value).onTrue(new RunCommand(() -> resetOdometry()).withTimeout(0.01));
    new JoystickButton(driver, XboxController.Button.kY.value).whileTrue(new TurnToSpeaker(driveTrain));

    // OPERATOR BINDINGS
    new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(new SpeakerShot(shooter, indexer, driveTrain).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
    new JoystickButton(operator, XboxController.Button.kB.value).whileTrue(new BumbperShot(shooter, indexer, driveTrain).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
    new JoystickButton(operator, XboxController.Button.kX.value).whileTrue(new Collect(acq, indexer, shooter));
    new JoystickButton(operator, XboxController.Button.kX.value).onFalse(new CenterNote(shooter, indexer));
    new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(new Reject(acq, indexer, shooter));
    new JoystickButton(operator, XboxController.Button.kStart.value).onTrue(new AmpShot(shooter, driveTrain, indexer));
    new JoystickButton(operator, XboxController.Button.kRightBumper.value).onTrue(new WindUp(ShooterState.maxSpeed, shooter));
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value).onTrue(new WindUp(ShooterState.notSpinning, shooter));
    new JoystickButton(operator, XboxController.Button.kLeftStick.value).onTrue(new RunCommand(() -> CommandScheduler.getInstance().cancelAll()));
  }

  /**
   * Returns the currently selected autonomous command.
   * @return the currently selected autonomous command
   */
  public Command getAutonomousCommand() {
    controlState = ControlState.AUTO;
      driveTrain.setInitPose(
        getAlliance().equals(Alliance.Blue)
        ? new Pose2d(startingPos.getSelected().getTranslation(), new Rotation2d(-startingPos.getSelected().getRotation().getRadians()))
        : new Pose2d(new Translation2d(Constants.Field.fieldWidth - startingPos.getSelected().getX(), startingPos.getSelected().getY()), new Rotation2d(Math.PI + startingPos.getSelected().getRotation().getRadians()))
      );

    if (autoCommandChooser.getSelected().equals("OneRing")) {
      double finalAngle = Math.atan2(ringPositionChooser.getSelected().getY() - startingPos.getSelected().getY(), ringPositionChooser.getSelected().getX() - startingPos.getSelected().getX());
      return new SequentialCommandGroup(
            new BumbperShot(shooter, indexer, driveTrain),
            new ParallelRaceGroup
            (
              new DriveToPoint(driveTrain, new Pose2d(ringPositionChooser.getSelected().getTranslation(), new Rotation2d(getAlliance().equals(Alliance.Blue) ? finalAngle : Math.PI - finalAngle))),
              new Collect(acq, indexer, shooter),
              new WaitCommand(2)
            ),
            new TurnToSpeaker(driveTrain),
            new SpeakerShot(shooter, indexer, driveTrain)
            );

    } else if (autoCommandChooser.getSelected().equals("Shoot")) {
      return new BumbperShot(shooter, indexer, driveTrain);
    } else if (autoCommandChooser.getSelected().equals("Nothing")) {
      return null;
    } else if (autoCommandChooser.getSelected().equals("Mobility")) {
      return new SequentialCommandGroup(
        new BumbperShot(shooter, indexer, driveTrain),
        new DriveToPoint(driveTrain, new Pose2d(new Translation2d(getAlliance().equals(Alliance.Blue) ? 2.4 : Constants.Field.fieldWidth - 2.4, 1.22), new Rotation2d(getAlliance().equals(Alliance.Blue) ? 0 : Math.PI)))
      );
    } else if (autoCommandChooser.getSelected().equals("Mess With 2 Notes")) {
      return new SequentialCommandGroup(
        new BumbperShot(shooter, indexer, driveTrain),
        new WaitCommand(6),
        new PathPlannerAuto("auto1")
      );
    } else if (autoCommandChooser.getSelected().equals("Mess With Note")) {
      return new SequentialCommandGroup(
        new BumbperShot(shooter, indexer, driveTrain),
        new WaitCommand(6),
        new DriveToPoint(driveTrain, new Pose2d(ringPositionChooser.getSelected().getTranslation(), new Rotation2d(Math.PI / 4)))
      );
    } else {
      return null;
    }
  }

  /**
   * returns the currently selected drive command
   * @return
   */
  public Command getDriveCommand() {
    controlState = ControlState.MANUAL;
    Command driveCommand;
    switch (inputMode.getSelected()) {
      default:
        driveCommand = new JoystickDrive(driveTrain, driver);
        break;
      case "controller":
        driveCommand = new JoystickDrive(driveTrain, driver);
        break;
      case "ddr":
        driveCommand = new DDRDrive(driveTrain);
        break;
      case "demo":
        driveCommand = new ManualControl(driveTrain, shooter);
        break;
    }
    return driveCommand;
  }

  /**
   * gets the current alliance, defaults to blue if it is called before field side is set
   * @return the current alliance
   */
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
    return Alliance.Blue;
  }

  /**
   * Sets the field relative direction to forward if it doesn't do it in auto
   */
  public void resetOdometry() {
    driveTrain.resetOdometry(new Pose2d());
    driveTrain.zeroHeading();
    driveTrain.setGyroAngle(0);
  }

  /**
   * gets the shooter state
   * @return the shooter state
   */
  public static ShooterState getShooterState() {
      return shooterState;
  }
  
  /**
   * sets the shooter state
   * @param shooterState the shooter state
   */
  public static void setShooterState(ShooterState shooterState) {
      RobotContainer.shooterState = shooterState;
  }

  /** AUTO, PATHING, MANUAL */
  public enum ControlState {
    AUTO, PATHING, MANUAL
  }

  /** IDLE, DRIVING, COLLECTING, CENTERING, HAS_NOTE, SHOOTING, CLIMBING */
  public enum ActivityState {
    IDLE, DRIVING, COLLECTING, CENTERING, HAS_NOTE, SHOOTING, CLIMBING
  }

  /** Only use to drive LEDs, not safe for anything else */
  public static ControlState controlState;
  /** Only use to drive LEDs, not safe for anything else */
  public static ActivityState activityState;

  public static SendableChooser<Integer> blinkTickSpeedChooser = new SendableChooser<>();
  public static SendableChooser<Integer> rainbowTickSpeedChooser = new SendableChooser<>();
  static {
    rainbowTickSpeedChooser.addOption("ten", 10);
    rainbowTickSpeedChooser.addOption("five", 5);
    rainbowTickSpeedChooser.addOption("one", 1);
    rainbowTickSpeedChooser.setDefaultOption("one", 1);
    SmartDashboard.putData(rainbowTickSpeedChooser);

    blinkTickSpeedChooser.addOption("ten", 10);
    blinkTickSpeedChooser.addOption("five", 5);
    blinkTickSpeedChooser.addOption("one", 1);
    blinkTickSpeedChooser.setDefaultOption("one", 1);
    SmartDashboard.putData(blinkTickSpeedChooser);
  }

  public static void setLEDs(Color color) {LED.setColor(color);}
}
