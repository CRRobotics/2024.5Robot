package frc.robot.util;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public interface Constants {

    //SUBSYSTEMS
    
    interface DriveTrain {
        int frontLeftWheelID = 2;
        int frontLeftTurnID = 1;
        double frontLeftAngularOffset = -Math.PI / 2;

        int frontRightWheelID = 8;
        int frontRightTurnID = 7;
        double frontRightAngularOffset = 0;

        int backLeftWheelID = 4;
        int backLeftTurnID = 3;
        double backLeftAngularOffset = Math.PI;

        int backRightWheelID = 6;
        int backRightTurnID = 5;
        double backRightAngularOffset = Math.PI / 2;

        /** Distance between centers of right and left wheels on robot.
         * <p>//TODO: Set kTrackWidth to actual track width */
        double trackWidth = Units.inchesToMeters(26.5);
        /** Distance between front and back wheels on robot.
         * <p>//TODO: Set kWheelBase to actual wheel base */
        double wheelBase = Units.inchesToMeters(26.5);
        SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2),
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2));
        /** Swerve Max Speed in m/s.
         * <p>(copied from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java) */
        double maxSpeed = 5;
        double maxAcceleration = 4;
        /** radians per second */
        double maxAngularSpeed = 2 * Math.PI;
        /** radians per second squared */
        double maxAngularAcceleration = 4 * Math.PI;

        /** possibly determines whether gyro is reversed */
        boolean gyroReversed = true;

        double driveDeadBand = 0.025;

        /** radians per second */
        double magnitudeSlewRate = 2.4;
        /** radians per second (1 = 100%) */
        double kDirectionSlewRate = 2.5;
        /** percent per second (1 = 100%) */
        double rotationSlewRate = 2.2;
        /** radius in meters (of what I have no idea) */
        double radius = 0.4318;

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

    interface Indexer {
        int indexID = 12;
        /** units? */
        double indexIntakeSpeed = -0.68;
        /** units? */
        double indexRejectSpeed = 0.3;
        /** units? */
        double indexShootSpeed = -4;
    }
    
    interface Intake {
        int intakeID = 13;
        /** units? */
        double intakeCollectSpeed = -0.3;
        /** units? */
        double intakeRejectSpeed = 0.5;
    }

    interface Shooter {
        // FLYWHEELS (Krakens)
        int leftShooterMotorID = 14;
        int rightShooterMotorID = 15;

        /** Belt between krakens and shooter flywheels */
        double krakenePulleyTeethNum = 16;
        double flywheelPulleyTeethNum = 24;
        double beltRatio = flywheelPulleyTeethNum / krakenePulleyTeethNum;
        
        double outdexTime = 200;
        double conveyTime = 400;
        double spinUpTime = 0;
        double shootTime = 1200;

        double krakenP = 0.09;
        double krakenI = 0.0;
        double krakenD = 0.0;
        /** Krakens */
        double talonControllerAcceleration = 380;
        /** Krakens */
        double voltageControllerVelocity = 12;
        /** feed forward for kraken*/ 
        double kV = .12;
        /** Speed used when <code>WindUp.java</code> just wants to shoot as hard as possible */
        double shooterDefaultMaxSpeed = 420;



        // AZIMUTH (Neos)
        int pivotMotorID = 11;

        double restAngle = 4.3;
        /** angle at which intake feeds indexer nicely */
        double interfaceAngle = 4.46;
        /** angle at which indexer can push the note back into the intake nicely */
        double rejectInterfaceAngle = 4.43;
        /** tolerence for interface angle */
        double interfaceError = .08;
        /** Angle area where the LimeLight could interfiere with the note in the indexer if the note isnt all the way inside
         * <p>TODO: Find the perfect value */
        double limeLightWarningZone = 4.9;

        /** Angle setpoint for the amp
         * <p>TODO: Tune this shi */
        double ampSetPoint = 6.0;// TODO: TUNE THIS
        /** Angle setpoint for the amp
         * <p>TODO: Tune this shi */
        double ampShotSpeed = 30;// TODO: TUNE THIS


        // Systems controll parameters
        double sparkP = 0.0085;
        double sparkI = 0;
        double sparkD = 0;
        double sparkFF = 0.00;

        /** slotID for Neo SmartMotion */
        int slotID = 0;
        /** For the azimuth neos */
        double smartMotionMaxVelocity = 120;
        /** For the azimuth neos */
        double smartMotionMinVelocity = 0;
        /** For the azimuth neos */
        double smartMotionMaxAccel = 90;
        /** For the azimuth neos */
        double smartMotionAllowedClosedLoopError = 0;
    }
    
    interface Winch {
        int leftID = 9;
        int rightID = 10;

        double extendSpeed = 0.2;
        double retractSpeed = 0.9;
        double extendTime = 2000;
        // double retractTime = 600;

        double currentDifferenceThreshold = 7000; //TODO: Tune value

        double winchP = 0;;
        double winchI = 0;
        double winchD = 0;
        double winchFF = 0;

        //for PID
        int slotID = 1;
        double MaxVelocity = 120;
        double MinVelocity = 0;
        double MaxAccel = 1;
        double AllowedClosedLoopError = 0;

    }

    // OTHER STUFF

    interface Auto {
        double maxSpeed = 2; // meters per second
        double maxAcceleration = 2; // meters per second squared
        double maxAngularSpeed = 2 * Math.PI; // radians per second
        double maxAngularAcceleration = 2 * Math.PI; // radians per second squared
        double thetaP = 1; // pids for auto
        double xP = 1;
        double yP = 1;
        TrapezoidProfile.Constraints thetaPIDConstraints = new TrapezoidProfile.Constraints(
            maxAngularSpeed, maxAngularAcceleration);
        double balanceP = 0;
        double balanceI = 0;
        double balanceD = 0;
        double balanceTolerance = 5;
        double testMult = 1;
        PathConstraints constraints = new PathConstraints(
            maxSpeed * testMult,
            maxAcceleration * testMult,
            maxAngularSpeed * testMult / 2,
            maxAcceleration * testMult / 2
        );

        interface TurnToAngle {
            double kP = 2.2;
            double kI = 0;
            double kD = 0;
        }
        
        /*
         * Note positions for auto
         */
        interface NotePositions {
            //Throw these points as end points into path planner
            //trust, it's dead accurate
            //move the bot center ontop a ring
            Translation2d[] kNotesStartingMidline = {
                new Translation2d(8.258, 7.462),
                new Translation2d(8.258, 5.785),
                new Translation2d(8.258, 4.109),
                new Translation2d(8.258, 2.432),
                new Translation2d(8.258, 0.756),
            };

            Translation2d[] kNotesStartingBlueWing = {
                new Translation2d(2.884, 4.109),
                new Translation2d(2.884, 5.557),
                new Translation2d(2.884, 7.004),
            };

            Translation2d[] kNotesStartingRedWing = {
                new Translation2d(13.63, 4.109),
                new Translation2d(13.63, 5.557),
                new Translation2d(13.63, 7.004),
            };
        }
    }

    interface Controller {
        int driverControllerPort = 0;
        int operatorControllerPort = 1;
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

        Translation2d ampBlue = tag6.minus(new Translation2d(0, DriveTrain.trackWidth / 2));
        Translation2d subwooferBlue = new Translation2d(1.24, 5.553456);
        Translation2d speakerBlue = tag7.plus(new Translation2d(DriveTrain.trackWidth, 0));

        Translation2d ampRed = tag5.minus(new Translation2d(0, DriveTrain.trackWidth / 2));
        Translation2d subwooferRed = new Translation2d(fieldWidth - subwooferBlue.getX(), subwooferBlue.getY());
        Translation2d speakerRed = tag4.minus(new Translation2d(DriveTrain.trackWidth, 0));

    
    }
    
    /**
     * Just used in <code>SwerveModule.java</code>
     */
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
