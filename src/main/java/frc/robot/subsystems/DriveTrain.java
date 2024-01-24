// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import java.util.stream.IntStream;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.NetworkTableWrapper;
import frc.robot.util.SwerveModule;
public class DriveTrain extends SubsystemBase implements Constants.Drive, Constants.Drive.PoseEstimator {
    // Create MAXSwerveModules
    public final SwerveModule frontLeft = new SwerveModule( // chimera 11& 12
            Constants.Drive.chimeraWheelID,
            Constants.Drive.chimeraTurnID,
            Constants.Drive.frontLeftAngularOffset);

    public final SwerveModule frontRight = new SwerveModule( // manticore 9&10
            Constants.Drive.manticoreWheelID,
            Constants.Drive.manticoreTurnID,
            Constants.Drive.frontRightAngularOffset);

    public final SwerveModule backLeft = new SwerveModule( //phoenix 13&14
            Constants.Drive.phoenixWheelID,
            Constants.Drive.phoenixTurnID,
            Constants.Drive.backLeftAngularOffset);

    public final SwerveModule backRight = new SwerveModule( //Leviathan 5&6
            Constants.Drive.leviathanWheelID,
            Constants.Drive.leviathanTurnID,
            Constants.Drive.backRightAngularOffset);
// Cerberus 7&8
    // The gyro sensor
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private Pathfinder pathFinder;
    
    
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
        new Pose2d(0, 0, Rotation2d.fromRadians(Math.PI)), // initial pose
        VecBuilder.fill(stateTrans, stateTrans, stateTheta), // odometry standard deviation for x, y, theta
        VecBuilder.fill(visionTrans, visionTrans, visionTheta) // visions standard deviation for x, y, theta
    );

    private Field2d field = new Field2d();
    // private Field2d odoField = new Field2d();

    /** Creates a new DriveSubsystem. */
    public DriveTrain() {
        // resetOdometry(new Pose2d(1.6, 4.4, Rotation2d.fromRadians(2.8)));
        zeroHeading();

        AutoBuilder.configureHolonomic(
            this::getPose, // Pose supplier
            this::resetOdometry, // SwerveDriveKinematics
            this::getChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(true, false, 0.5, 0.2) // Default path replanning config. See the API for the options here
            ),
            this::flipPath,
            this // Reference to this subsystem to set requirements
        );

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });

        Pathfinding.setPathfinder(pathFinder = new LocalADStarAK());
    }

    @Override
    public void periodic() {
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
        Translation2d tag8 = new Translation2d(0, 5);
        for (String id : cameraIds) {
            double x = NetworkTableWrapper.getDouble(id, "rx");
            double y = NetworkTableWrapper.getDouble(id, "ry");
            double ntags = NetworkTableWrapper.getDouble(id, "ntags");
            double theta = NetworkTableWrapper.getDouble(id, "theta");
            if (ntags != 0 && x != cameraErrorCode) {
                double distance = tag8.getDistance(new Translation2d(x, y));
                SmartDashboard.putNumber("distance to tag", distance);
                poseEstimator.addVisionMeasurement(
                    new Pose2d(x, y, Rotation2d.fromRadians(theta)),
                    Timer.getFPGATimestamp() + 0.01, // needs to be tested and calibrated
                    VecBuilder.fill(1 * distance, 1 * distance, 1 * distance) // needs to be calibrated
                );
            }
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
    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        poseEstimator.resetPosition(
                rotation,
                new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
                },
                pose);
    }

     /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        setGyroAngle(pose.getRotation().getRadians());
        poseEstimator.resetPosition(
                gyro.getRotation2d(),
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
        gyro.setAngleAdjustment(angle * 180 / Math.PI);
    }

    public double getGyroAngle() {
        return -gyro.getAngle() * Math.PI / 180;
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in radians
     */
    public double getHeading() {
        return poseEstimator.getEstimatedPosition().getRotation().getRadians();
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

    /*
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
        Command driveCommand = followPathCommand(PathPlanner.generatePath(
            new PathConstraints(0.5, 0.5),
            new PathPoint(robotPosition, translationRotation, getPose().getRotation()), // starting pose
            new PathPoint(targetPose.getTranslation(), translationRotation, Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Red ? 0 : 180))), // ending pose
            false);
        return driveCommand;
    } */

    /*
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
        // Command driveCommand = followPathCommand(PathPlanner.generatePath(
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
        Command driveCommand = followPathCommand(PathPlanner.generatePath(
            new PathConstraints(0.5, 0.5),
            new PathPoint(robotPosition, translationRotation, getPose().getRotation()), // starting pose
            new PathPoint(targetPose.getTranslation(), translationRotation, Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Red ? 0 : 180))), // ending pose
            false);
        return driveCommand;
    } */

    public Command followPathCommand(PathPlannerPath path) {
        PIDController thetaController = new PIDController(
            SmartDashboard.getNumber("drivetrain/thetaP", 0),
            SmartDashboard.getNumber("drivetrain/thetaI", 0),
            SmartDashboard.getNumber("drivetrain/thetaD", 0)
        ); // Rotation PID controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(0.025);
        SmartDashboard.putNumber("theta position tolerance", thetaController.getPositionTolerance());
        SmartDashboard.putNumber("theta velocity tolerance", thetaController.getVelocityTolerance());
        // field.getObject("traj").setTrajectory(traj);
        return new FollowPathHolonomic(
                path, 
                this::getPose, // Pose supplier
                this::getChassisSpeeds, // SwerveDriveKinematics
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(SmartDashboard.getNumber("drivetrain/xP", 0), SmartDashboard.getNumber("drivetrain/xI", 0), SmartDashboard.getNumber("drivetrain/xD", 0)), // Translation PID constants
                    new PIDConstants(SmartDashboard.getNumber("drivetrain/xP", 0), SmartDashboard.getNumber("drivetrain/xI", 0), SmartDashboard.getNumber("drivetrain/xD", 0)), // Translation PID constants
                Constants.Drive.maxSpeed, // Max module speed, in m/s
                Constants.Drive.radius, // Drive base radius in meters. Distance from robot center to furthest module.,
                    new ReplanningConfig(true, false, 0.5, 0.2)), // Default path replanning config. See the API for the options here
                this::flipPath,
                this // Reference to this subsystem to set requirements
        );
    }

    public ChassisSpeeds getChassisSpeeds(){
        return driveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = driveKinematics.toSwerveModuleStates(targetSpeeds);
        this.setModuleStates(targetStates);
        this.updateObstacles();
      }

    public boolean flipPath(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public void updateObstacles(){
        // NetworkTableWrapper.getDouble(i, "rx");
        ArrayList<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<Pair<Translation2d, Translation2d>>();
        obstacles.add(new Pair(new Translation2d(1, -1), new Translation2d(2, 1)));
        pathFinder.setDynamicObstacles(obstacles, this.getPose().getTranslation());
    }

    private Command FollowPathWithEvents(FollowPathHolonomic followPathHolonomic) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'FollowPathWithEvents'");
    }
}