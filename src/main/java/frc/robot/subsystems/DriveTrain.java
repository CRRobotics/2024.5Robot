// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.NetworkTableWrapper;
import frc.robot.util.SwerveModule;
public class DriveTrain extends SubsystemBase implements Constants.Drive {
    // Create MAXSwerveModules
    private final SwerveModule frontLeft = new SwerveModule( // chimera 11& 12
            Constants.Drive.chimeraWheelID,
            Constants.Drive.chimeraTurnID,
            Constants.Drive.frontLeftAngularOffset);

    private final SwerveModule frontRight = new SwerveModule( // manticore 9&10
            Constants.Drive.manticoreWheelID,
            Constants.Drive.manticoreTurnID,
            Constants.Drive.frontRightAngularOffset);

    private final SwerveModule backLeft = new SwerveModule( //phoenix 13&14
            Constants.Drive.phoenixWheelID,
            Constants.Drive.phoenixTurnID,
            Constants.Drive.backLeftAngularOffset);

    private final SwerveModule backRight = new SwerveModule( //Leviathan 5&6
            Constants.Drive.leviathanWheelID,
            Constants.Drive.leviathanTurnID,
            Constants.Drive.backRightAngularOffset);
// Cerberus 7&8
    // The gyro sensor
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        Constants.Drive.driveKinematics,
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        }
    );
    
    // Kalman filter for tracking robot pose
    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Drive.driveKinematics, // kinematics
        Rotation2d.fromDegrees(-gyro.getAngle()), // initial angle
        new SwerveModulePosition[] { // initial module positions
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        new Pose2d(1.81, 0.45, Rotation2d.fromRadians(Math.PI)), // initial pose
        VecBuilder.fill(0.1, 0.1, 0.1), // odometry standard deviation for x, y, theta
        VecBuilder.fill(0.5, 0.5, 0.5) // visions standard deviation for x, y, theta
    );

    private Field2d field = new Field2d();
    // private Field2d odoField = new Field2d();

    /** Creates a new DriveSubsystem. */
    public DriveTrain() {
        // resetOdometry(new Pose2d(1.6, 4.4, Rotation2d.fromRadians(2.8)));
        zeroHeading();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pitch", getPitch());
        
        double xTarget = DriverStation.getAlliance().equals(Alliance.Red) ? 14.73 : 1.82;
        double[] scoringPositions = {
            0.46, 1.07, 1.64, 2.2, 2.74, 1.81, 3.87, 4.42, 5.07 // y positions in m of 9 scoring positions
        };
        Translation2d robotPosition = getPose().getTranslation(); // current position

        // calculates which position is closest
        double[] distances = new double[9];
        int minDistanceIndex = 0;
        for (int i = 0; i < distances.length; i++) {
            distances[i] = robotPosition.getDistance(new Translation2d(xTarget, scoringPositions[i]));
            if (distances[i] < distances[minDistanceIndex]) {
                minDistanceIndex = i;
            }
        }
        SmartDashboard.putNumber("closest scoring position", minDistanceIndex);
        SmartDashboard.putNumber("distanceToPosition", robotPosition.getDistance(new Translation2d(xTarget, scoringPositions[minDistanceIndex])));


        SmartDashboard.putNumber("gyro angle", gyro.getAngle());
        SwerveModulePosition[] swervePosition = {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
        
        poseEstimator.update(
            Rotation2d.fromDegrees(-gyro.getAngle()),
            swervePosition
        );

        odometry.update(
            Rotation2d.fromDegrees(-gyro.getAngle()),
            swervePosition
        );
        
        // update with visions data from these cameras ids:
        for (String i : new String[]{"0", "2", "4"}) {
            if (NetworkTableWrapper.getDouble(i, "ntags") != 0 && NetworkTableWrapper.getDouble(i, "rx") < 20 && NetworkTableWrapper.getDouble(i, "ry") < 20) {
                double distance = getPose().getTranslation().getDistance(new Translation2d(NetworkTableWrapper.getDouble(i, "rx"), NetworkTableWrapper.getDouble(i, "ry")));
                poseEstimator.addVisionMeasurement(
                    new Pose2d(
                        NetworkTableWrapper.getDouble(i, "rx"),
                        NetworkTableWrapper.getDouble(i, "ry"),
                        Rotation2d.fromRadians(NetworkTableWrapper.getDouble(i, "theta"))
                    ),
                    Timer.getFPGATimestamp() + 0.01, // needs to be tested and calibrated
                    VecBuilder.fill(0.8 * distance, 0.8 * distance, 0.8 * distance) // needs to be calibrated
                );
            }
            SmartDashboard.putNumber(i + "Theta", NetworkTableWrapper.getDouble(i, "theta") * 180 / Math.PI);
        }

        // field
        // odoField.setRobotPose(odometry.getPoseMeters());
        // SmartDashboard.putData(odoField);
        field.setRobotPose(getPose());
        SmartDashboard.putData(field);
    }

    public Field2d getField() {
        return field;
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                Rotation2d.fromDegrees(gyro.getAngle()),
                new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
                },
                pose);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        SmartDashboard.putNumber("drive/wheel angle", desiredStates[0].angle.getDegrees());
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Drive.maxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setMotorSpeeds(double speed) {
        setModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(speed, new Rotation2d(0)),
            new SwerveModuleState(speed, new Rotation2d(0)),
            new SwerveModuleState(speed, new Rotation2d(0)),
            new SwerveModuleState(speed, new Rotation2d(0))
        });
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    public void setGyroAngle(double angle) {
        // gyro.setAngleAdjustment(angle * 180 / Math.PI);
        gyro.setAngleAdjustment(getHeading());
    }

    public double getGyroAngle() {
        return -gyro.getAngle() * Math.PI / 180;
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return states;
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    /**
     * Returns the turn rate of the robot from the gyroscope.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (Constants.Drive.gyroReversed ? -1.0 : 1.0);
    }

    public Command closestScoringCommand() {
        double xTarget = DriverStation.getAlliance().equals(Alliance.Red) ? 14.73 : 1.82;
        double[] scoringPositions = {
            0.46, 1.07, 1.64, 2.2, 2.74, 3.33, 3.87, 4.42, 5.07 // y positions in m of 9 scoring positions
        };
        Translation2d robotPosition = getPose().getTranslation(); // current position

        // calculates which position is closest
        double[] distances = new double[9];
        int minDistanceIndex = 0;
        for (int i = 0; i < distances.length; i++) {
            distances[i] = robotPosition.getDistance(new Translation2d(xTarget, scoringPositions[i]));
            if (distances[i] < distances[minDistanceIndex]) {
                minDistanceIndex = i;
            }
        }
        //minDistanceIndex = 5;
        SmartDashboard.putNumber("closest scoring position", minDistanceIndex);

        Pose2d targetPose = new Pose2d(new Translation2d(xTarget, scoringPositions[minDistanceIndex]), Rotation2d.fromDegrees(0)); // top node on red
        Translation2d translationDifference = targetPose.getTranslation().minus(robotPosition); // difference between target and current
        // calculates wheel angle needed to target from x and y components
        Rotation2d translationRotation = new Rotation2d(translationDifference.getX(), translationDifference.getY());
        Command driveCommand = followTrajectoryCommand(PathPlanner.generatePath(
            new PathConstraints(0.5, 0.5),
            new PathPoint(robotPosition, translationRotation, getPose().getRotation()), // starting pose
            new PathPoint(targetPose.getTranslation(), translationRotation, Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Red ? 0 : 180))), // ending pose
            false);
        return driveCommand;
    }

    public Command driveToPieceCommand() {
        // double[] pieceData;
        // if (NetworkTableWrapper.getArray("Detector", "Cone")[0] == 1) {
        //     pieceData = NetworkTableWrapper.getArray("Detector", "Cone");
        // } else {
        //     pieceData = NetworkTableWrapper.getArray("Detector", "Cube");
        // }
        // Translation2d currentPosition = getPose().getTranslation();

        // double xCoordinateOfRobot = currentPosition.getX();
        // double yCoordinateOfRobot = currentPosition.getY();
        // double rotationAngleOfRobot = getPose().getRotation().getRadians();
        // GetGlobalCoordinates myGlobalCoordinates = new GetGlobalCoordinates(xCoordinateOfRobot, yCoordinateOfRobot, rotationAngleOfRobot, pieceData);
        // double targetX = myGlobalCoordinates.globalX + (DriverStation.getAlliance() == Alliance.Blue ? -0.77 : 0.77);
        // double targetY = myGlobalCoordinates.globalY;

        // Rotation2d translationRotation = new Rotation2d(targetX, targetY);
        // SmartDashboard.putNumber("target X", targetX);
        // SmartDashboard.putNumber("target Y", targetY);
        // Command driveCommand = followTrajectoryCommand(PathPlanner.generatePath(
        //     new PathConstraints(0.5, 0.5),
        //     new PathPoint(getPose().getTranslation(), translationRotation, getPose().getRotation()),
        //     new PathPoint(new Translation2d(targetX, targetY), translationRotation, Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Blue ? 0 : 180))
        //     // new PathPoint(getPose().getTranslation(), translationRotation, getPose().getRotation())
        //     ),
        //     false);
        // field.getObject("traj").setTrajectory(PathPlanner.generatePath(
        //     new PathConstraints(0.5, 0.5),
        //     new PathPoint(getPose().getTranslation(), translationRotation, getPose().getRotation()),
        //     new PathPoint(new Translation2d(targetX, targetY), translationRotation, Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Blue ? 0 : 180))
        //     // new PathPoint(getPose().getTranslation(), translationRotation, getPose().getRotation())
        //     ));
        // return driveCommand;
        double xTarget = DriverStation.getAlliance().equals(Alliance.Red) ? 0.72 : 15.8;
        double[] scoringPositions = {
            6.15, 7.48 // y positions in m of 9 scoring positions
        };
        Translation2d robotPosition = getPose().getTranslation(); // current position

        // calculates which position is closest
        double[] distances = new double[2];
        int minDistanceIndex = 0;
        for (int i = 0; i < distances.length; i++) {
            distances[i] = robotPosition.getDistance(new Translation2d(xTarget, scoringPositions[i]));
            if (distances[i] < distances[minDistanceIndex]) {
                minDistanceIndex = i;
            }
        }
        //minDistanceIndex = 5;
        SmartDashboard.putNumber("closest scoring position", minDistanceIndex);

        Pose2d targetPose = new Pose2d(new Translation2d(xTarget, scoringPositions[minDistanceIndex]), Rotation2d.fromDegrees(0)); // top node on red
        Translation2d translationDifference = targetPose.getTranslation().minus(robotPosition); // difference between target and current
        // calculates wheel angle needed to target from x and y components
        Rotation2d translationRotation = new Rotation2d(translationDifference.getX(), translationDifference.getY());
        Command driveCommand = followTrajectoryCommand(PathPlanner.generatePath(
            new PathConstraints(0.5, 0.5),
            new PathPoint(robotPosition, translationRotation, getPose().getRotation()), // starting pose
            new PathPoint(targetPose.getTranslation(), translationRotation, Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Red ? 0 : 180))), // ending pose
            false);
        return driveCommand;
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean mirrorPath) {
        PIDController thetaController = new PIDController(
            SmartDashboard.getNumber("drivetrain/thetaP", 0),
            SmartDashboard.getNumber("drivetrain/thetaI", 0),
            SmartDashboard.getNumber("drivetrain/thetaD", 0)
        ); // Rotation PID controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(0.025);
        SmartDashboard.putNumber("theta position tolerance", thetaController.getPositionTolerance());
        SmartDashboard.putNumber("theta velocity tolerance", thetaController.getVelocityTolerance());
        field.getObject("traj").setTrajectory(traj);
        return new SequentialCommandGroup(
                     new PPSwerveControllerCommand(
                        traj, 
                        this::getPose, // Pose supplier
                        driveKinematics, // SwerveDriveKinematics
                        new PIDController(
                            SmartDashboard.getNumber("drivetrain/xP", 0),
                            SmartDashboard.getNumber("drivetrain/xI", 0),
                            SmartDashboard.getNumber("drivetrain/xD", 0)
                        ), // X PID controller
                        new PIDController(
                            SmartDashboard.getNumber("drivetrain/xP", 0),
                            SmartDashboard.getNumber("drivetrain/xI", 0),
                            SmartDashboard.getNumber("drivetrain/xD", 0)
                        ), // Y PID controller, probably the same as X controller
                        thetaController,
                        this::setModuleStates, // Module states consumer
                        mirrorPath, // mirrors path based on alliance
                        this // Requires this drive subsystem
                     )
        );
    }
}