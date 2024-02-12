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

    interface Acquisition {
        int aqID = 0;
        double aqIntakeSpeed = 0;
        double aqRejectSpeed = 0;
    }

    interface Auto {
        double maxSpeed = 3; // meters per second
        double maxAcceleration = 4; // meters per second squared
        double maxAngularSpeed = Math.PI; // radians per second
        double maxAngularAcceleration = Math.PI; // radians per second squared
        double thetaP = 1; // pids for auto
        double xP = 1;
        double yP = 1;
        TrapezoidProfile.Constraints thetaPIDConstraints = new TrapezoidProfile.Constraints(
            maxAngularSpeed, maxAngularAcceleration);
        double balanceP = 0;
        double balanceI = 0;
        double balanceD = 0;
        double balanceTolerance = 5;
        PathConstraints constraints = new PathConstraints(maxSpeed, maxAcceleration, maxAngularSpeed, maxAcceleration);
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
    }


    interface Field {
        double fieldWidth = 16.54;

        Translation2d tag1 = new Translation2d(15.079472, 0.245872);
        Translation2d tag2 = new Translation2d(16.185134, 0.883666);
        Translation2d tag3 = new Translation2d(16.579342, 4.982718);
        Translation2d tag4 = new Translation2d(16.579342, 5.547868);
        Translation2d tag5 = new Translation2d(14.700758, 8.2042);
        Translation2d tag6 = new Translation2d(1.8415, 8.2042);
        Translation2d tag7 = new Translation2d(-0.0381, 5.547868);
        Translation2d tag8 = new Translation2d(-0.0381, 4.982718);
        Translation2d tag9 = new Translation2d(0.356108, 0.883666);
        Translation2d tag10 = new Translation2d(1.461516, 0.245872);
        Translation2d tag11 = new Translation2d(11.904726, 3.713226);
        Translation2d tag12 = new Translation2d(11.904726, 4.49834);
        Translation2d tag13 = new Translation2d(11.220196, 4.105148);
        Translation2d tag14 = new Translation2d(5.320792, 4.105148);
        Translation2d tag15 = new Translation2d(4.641342, 4.49834);
        Translation2d tag16 = new Translation2d(4.641342, 3.713226);

        Translation2d ampBlue = new Translation2d(1.93294, 4.867656);
        Translation2d speakerBlueBumper = new Translation2d(0.512318, 5.553456);
        Translation2d speakerBlueTarget = tag7; //maybe find a way to clone this
    }

    interface Shooter {
        double kF = 0;
        double restAngle = 0;
        double reverseIndexSpeed = 0;
        double reverseTime = 0;
        double spinUpTime = 0;
        double shootTime = 0;
    }
    interface Indexer {
        int indexID = 0;
        double indexIntakeSpeed = 0;
        double indexRejectSpeed = 0;
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

    interface Field{
        
    }
}
