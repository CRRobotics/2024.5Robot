// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.*;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.NetworkTableWrapper;
import frc.robot.util.SwerveModule;


public class DriveTrain extends SubsystemBase implements Constants.DriveTrain, Constants.DriveTrain.PoseEstimator {
    // SWERVE MODULES   spare module: // Cerberus 7&8
    public final SwerveModule frontLeft = new SwerveModule( // Chimera 11&12
        Constants.DriveTrain.frontLeftWheelID,
        Constants.DriveTrain.frontLeftTurnID,
        Constants.DriveTrain.frontLeftAngularOffset
    );
    public final SwerveModule frontRight = new SwerveModule( // Manticore 9&10
        Constants.DriveTrain.frontRightWheelID,
        Constants.DriveTrain.frontRightTurnID,
        Constants.DriveTrain.frontRightAngularOffset
    );
    public final SwerveModule backLeft = new SwerveModule( // Phoenix 13&14
        Constants.DriveTrain.backLeftWheelID,
        Constants.DriveTrain.backLeftTurnID,
        Constants.DriveTrain.backLeftAngularOffset
    );
    public final SwerveModule backRight = new SwerveModule( // Leviathan 5&6
        Constants.DriveTrain.backRightWheelID,
        Constants.DriveTrain.backRightTurnID,
        Constants.DriveTrain.backRightAngularOffset
    );

    RobotConfig config;
    DCMotor swerveMotor;

    // KINEMATICS
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private Pathfinder pathFinder;

    private Field2d field = new Field2d();
    // private Field2d odoField = new Field2d();
    
    // Odometry to keep track of the robot's movement history
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        Constants.DriveTrain.driveKinematics,
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        }
    );
    
    // Kalman filter for mixing odometry and vision data into final pose estimate
    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        Constants.DriveTrain.driveKinematics, // kinematics
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


    /**
     * Creates a new <code>DriveTrain</code> subsystem
     */
    public DriveTrain() {
        swerveMotor = new DCMotor(12, 2.6, 105, 1.8, (2*Math.PI) * 5676, 1);
        // resetOdometry(new Pose2d(1.6, 4.4, Rotation2d.fromRadians(2.8)));
        zeroHeading();

        config = new RobotConfig(25, 0, new ModuleConfig(.036, 4, 1, swerveMotor, 4.71, 30, 1), 0.637);
        AutoBuilder.configure(
            this::getPose, // Pose supplier
            this::resetOdometry, // SwerveDriveKinematics
            this::getChassisSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(10.0, 0.0, 0.0)), // Rotation PID constants
                    // Drive base radius in meters. Distance from robot center to furthest module.
            
            config,
            () -> {

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
            }, // Reference to this subsystem to set requirements
            this
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveTrain.maxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    /**
     * Sets the swerve module speed
     * 
     * @param speed The speed at which the motors are set to
     */
    public void setMotorSpeeds(double speed) {
        setModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(speed, new Rotation2d(0)),
            new SwerveModuleState(speed, new Rotation2d(0)),
            new SwerveModuleState(speed, new Rotation2d(0)),
            new SwerveModuleState(speed, new Rotation2d(0))
        });
    }

    /** 
     * Resets the drive encoders to currently read a position of 0. 
     */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    /** 
     * Zeroes the heading of the robot. 
     */
    public void zeroHeading() {
        gyro.reset();
    }
    /**
     * Sets the gyro angle adjustment to the specified angle in radians.
     * 
     * @param angle The angle in radians.
    */
    public void setGyroAngle(double angle) {
        gyro.setAngleAdjustment(angle * 180 / Math.PI);
    }
    /**
     * Returns the current gyro angle in radians.
     * 
     * @return The current gyro angle in radians.
     */
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
    /**
     * Returns swer module states
     * 
     * @return state of each of the swerve modules
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return states;
    }
    /**
     * 
     * @return current pitch value reported by the gyroscope
     */
    public double getPitch() {
        return gyro.getPitch();
    }
    /**
     * 
     * @return current roll value reported by the gyroscope
     */
    public double getRoll() {
        return gyro.getRoll();
    }
    
    /**
     * Returns the turn rate of the robot from the gyroscope.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (Constants.DriveTrain.gyroReversed ? -1.0 : 1.0);
    }

/**
 * Creates a command to follow a path using a combination of rotation and translation controls.
 * 
 * @param path The path to follow.
 * @return A command to follow the specified path.
 */
public Command followPathCommand(PathPlannerPath path) {
    // Rotation PID controller
    PIDController thetaController = new PIDController(
        SmartDashboard.getNumber("drivetrain/thetaP", 0.5),
        SmartDashboard.getNumber("drivetrain/thetaI", 0),
        SmartDashboard.getNumber("drivetrain/thetaD", 0)
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(0.025);

    // Display PID tolerances on SmartDashboard
    SmartDashboard.putNumber("theta position tolerance", thetaController.getPositionTolerance());
    SmartDashboard.putNumber("theta velocity tolerance", thetaController.getVelocityTolerance());

    // Create and return FollowPathHolonomic command
    return new FollowPathCommand(
        path,
        this::getPose, // Robot pose supplier
        this:: getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

    /**
     * Converts the state of each swerve module to chassis speeds.
     * 
     * @return the chasss speeds of the robopt
     */
    public ChassisSpeeds getChassisSpeeds(){
        return driveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }
    /**
     * Drives the robot relative to its current position using the given robot-relative chassis speeds.
     * 
     * @param robotRelativeSpeeds The robot-relative chassis speeds.
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = driveKinematics.toSwerveModuleStates(targetSpeeds);
        this.setModuleStates(targetStates);
        this.updateObstacles();
      }
    /**
     * Determines if the path should be flipped based on the current alliance color
     * 
     * @return True if the path should be flipped (alliance color is red), false otherwise.
     */
    public boolean flipPath(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
    /**
     * Gets the Pathfinder used for path planning.
     * 
     * @return The Pathfinder instance.
    */
    public Pathfinder getPathFinder(){
        return pathFinder;
    }
    
    /**
     * Updates the list of obstacles for path planning.
     * 
     * This method is currently commented out and not used until network tables are ready
     * @deprecated
     */
    public void updateObstacles(){
        // // NetworkTableWrapper.getDouble(i, "rx");
        // ArrayList<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<Pair<Translation2d, Translation2d>>();
        // obstacles.add(new Pair(new Translation2d(1, -1), new Translation2d(2, 1)));
        // pathFinder.setDynamicObstacles(obstacles, this.getPose().getTranslation());
        //Commented out until network tables are ready
    }

    private Command FollowPathWithEvents(FollowPathCommand followPathCommand) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'FollowPathWithEvents'");
    }
}