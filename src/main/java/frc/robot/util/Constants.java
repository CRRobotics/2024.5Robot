package frc.robot.util;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public interface Constants {

    interface Auto {
        double maxSpeed = 3; // meters per second
        double maxAcceleration = 4; // meters per second squared
        double maxAngularSpeed = 3 * Math.PI; // radians per second
        double maxAngularAcceleration = 3 * Math.PI; // radians per second squared
        double thetaP = 1; // pids for auto
        double xP = 1;
        double yP = 1;
        TrapezoidProfile.Constraints thetaPIDConstraints = new TrapezoidProfile.Constraints(
            maxAngularSpeed, maxAngularAcceleration);
        double balanceP = 0;
        double balanceI = 0;
        double balanceD = 0;
        double balanceTolerance = 5;
        double testMult = 0.25;
        PathConstraints constraints = new PathConstraints(
            maxSpeed * testMult,
            maxAcceleration * testMult,
            maxAngularSpeed * testMult,
            maxAcceleration * testMult
        );

        interface TurnToAngle {
            double kP = 2.2;
            double kI = 0;
            double kD = 0;
        }
    }

    interface Controller {
        int driveControllerPort = 0;
    }

    interface Drive {
        int chimeraWheelID = 11;
        int chimeraTurnID = 12;
        double frontLeftAngularOffset = -Math.PI / 2;

        int manticoreWheelID = 9;
        int manticoreTurnID = 10;
        double frontRightAngularOffset = 0;

        int phoenixWheelID = 13;
        int phoenixTurnID = 14;
        double backLeftAngularOffset = Math.PI;

        int leviathanWheelID = 5;
        int leviathanTurnID = 6;
        double backRightAngularOffset = Math.PI / 2;

        double trackWidth = Units.inchesToMeters(26.5);// Distance between centers of right and left wheels on robot//TODO Set kTrackWidth to actual track width
        double wheelBase = Units.inchesToMeters(26.5);// Distance between front and back wheels on robot //TODO Set kWheelBase to actual wheel base
        SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2, trackWidth / 2),
                new Translation2d(wheelBase / 2, -trackWidth / 2),
                new Translation2d(-wheelBase / 2, trackWidth / 2),
                new Translation2d(-wheelBase / 2, -trackWidth / 2));//Swerve Max Speed (copied from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java)
        double maxSpeed = 4; // meters per second
        double maxAcceleration = 4;
        double maxAngularSpeed = 2 * Math.PI; // radians per second;
        double maxAngularAcceleration = Math.PI; // radians per second squared
        boolean gyroReversed = true; //Determines whether the gyro is reversed (I think)

        double driveDeadBand = 0.025;

        double magnitudeSlewRate = 2.4; //rads per second
        double kDirectionSlewRate = 1.8; // percent per second (1 = 100%)
        double rotationSlewRate = 2.0; // percent per second (1 = 100%)
        double radius = 0.4318; //Radius in meters

        PathConstraints constraints = new PathConstraints(maxSpeed, maxAcceleration, maxAngularSpeed, maxAcceleration);

        double fastSpeedMultiplier = 1.4;
        double slowSpeedMultiplier = 0.25;

        String[] cameraIds = new String[]{"0", "2", "4", "6", "8", "10"};
        int cameraErrorCode = 63900;

        interface PoseEstimator {
            double stateTrans = 0.1;
            double stateTheta = 0.1;
            double visionTrans = 0.5;
            double visionTheta = 0.5;
        }
    }

    interface Field {
        double fieldWidth = 16.54;
        Translation2d ampBlue = new Translation2d(1.93294, 4.867656);
        Translation2d speakerBlue = new Translation2d(0.512318, 5.553456);

        Translation2d tag1 = new Translation2d(15.513558, 1.071626);
        Translation2d tag2 = new Translation2d(15.513558, 2.748026);
        Translation2d tag3 = new Translation2d(15.513558, 4.424426);
        Translation2d tag4 = new Translation2d(16.178784, 6.749796);
        Translation2d tag5 = new Translation2d(0.36195, 6.749796);
        Translation2d tag6 = new Translation2d(1.02743, 4.424426);
        Translation2d tag7 = new Translation2d(1.02743, 2.748026);
        Translation2d tag8 = new Translation2d(1.02743, 1.071626);
    }

    interface SwerveModule {
        double wheelDiameter = 0.0762; //meters
        double wheelCircumference = Math.PI * wheelDiameter; // meters
        double wheelTeeth = 14;
        // 45 teeth bevel gear, 22 teeth 1st stage spur gear, 15 teeth on bevel pinion gear
        double wheelGearRatio = (45.0 * 22) / (wheelTeeth * 15.0);
        double wheelEncoderPositionConversion = wheelDiameter * Math.PI / wheelGearRatio; // meters
        double wheelEncoderVelocityConversion = wheelEncoderPositionConversion / 60.0; // meters per second
        double wheelMotorFreeSpeed = 5676.0 / 60; // rotations per second
        double wheelFreeSpeed = (wheelMotorFreeSpeed * wheelCircumference) / wheelGearRatio; // rotations per second
        double wheelP = 0.04;
        double wheelI = 0;
        double wheelD = 0;
        double wheelFF = 0.22194;
        double wheelOutputMin = -1;
        double wheelOutputMax = 1;
        IdleMode wheelIdleMode = IdleMode.kBrake;
        int wheelCurrentLimit = 50; // amps

        boolean turnInverted = true;
        double turnEncoderPositionConversion = Math.PI * 2; // radians
        double turnEncoderVelocityConversion = Math.PI * 2 / 60; // radians per seconds
        double turnP = 1;
        double turnI = 0;
        double turnD = 0;
        double turnFF = 0;
        double turnOutputMin = -1;
        double turnOutputMax = 1;
        double turnEncoderPositionPIDMinInput = 0;
        double turnEncoderPositionPIDMaxInput = turnEncoderPositionConversion;
        IdleMode turnIdleMode = IdleMode.kBrake;
        int turnCurrentLimit = 20; // amps
    }
}
