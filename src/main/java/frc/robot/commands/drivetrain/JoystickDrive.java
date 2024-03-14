package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ActivityState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Constants;

/**
 * Runs the drivetrain
 */
public class JoystickDrive extends Command implements Constants.DriveTrain {
    DriveTrain driveTrain;
    XboxController controller;

    double currentRotation;
    double currentTranslationDir;
    double currentTranslationMag;
    SlewRateLimiter magnitudeLimiter;
    SlewRateLimiter rotationLimiter;
    double previousTime;

    private double speedAdjustedMaxSpeed;

    
    public JoystickDrive(DriveTrain driveTrain, XboxController controller) 
    {
        this.driveTrain = driveTrain;
        this.controller = controller;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        speedAdjustedMaxSpeed = maxSpeed;
        currentRotation = 0.0;
        currentTranslationDir = 0.0;
        currentTranslationMag = 0.0;
      
        magnitudeLimiter = new SlewRateLimiter(magnitudeSlewRate);
        rotationLimiter = new SlewRateLimiter(rotationSlewRate);
        previousTime = WPIUtilJNI.now() * 1e-6;
    }

    @Override
    public void execute() {
        RobotContainer.activityState = ActivityState.DRIVING;

        SmartDashboard.putString("Drive State", RobotContainer.driveStates.toString());
        switch(RobotContainer.driveStates)
        {
            case speeed:
                speedAdjustedMaxSpeed = maxSpeed * fastSpeedMultiplier;
                break;
            case normal:
                speedAdjustedMaxSpeed = maxSpeed;
                break;
            case slow:
                speedAdjustedMaxSpeed = maxSpeed * slowSpeedMultiplier;
                break;
        }
        SmartDashboard.putNumber("adjusted max speed", speedAdjustedMaxSpeed);

        double xSpeed = -MathUtil.applyDeadband(controller.getLeftY(), driveDeadBand);
        double ySpeed = -MathUtil.applyDeadband(controller.getLeftX(), driveDeadBand);
        double rotation = -MathUtil.applyDeadband(controller.getRightX(), driveDeadBand);
        // boolean fieldRelative = SmartDashboard.getBoolean("field relative", false);
        boolean fieldRelative = true;


        SmartDashboard.putNumber("xspeed", -controller.getLeftY());
        SmartDashboard.putNumber("ySpeed", -controller.getLeftX());
        SmartDashboard.putNumber("rotation", -controller.getRightX());

        double xSpeedCommanded;
        double ySpeedCommanded;

        // Convert XY to polar for rate limiting
        double inputDirection;
        if (ySpeed == 0 && xSpeed == 0) {
            inputDirection = currentTranslationDir;
        }
        else {
            inputDirection = Math.atan2(ySpeed, xSpeed);
        }
        double inputMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
        SmartDashboard.putNumber("inputmagnitude", inputMagnitude);

        // Calculate the direction slew rate based on an estimate of the lateral acceleration
        double directionSlewRate;
        SmartDashboard.putNumber("Current Translation Mag", currentTranslationMag);
        if (currentTranslationMag != 0.0) {
            directionSlewRate = Math.abs(kDirectionSlewRate / currentTranslationMag);
        } else {
            directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
        }
        

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - previousTime;
        double angleDif = angleDifference(inputDirection, currentTranslationDir);
        if (angleDif < 0.45 * Math.PI) {
            currentTranslationDir = stepTowardsCircular(currentTranslationDir, inputDirection, directionSlewRate * elapsedTime);
            currentTranslationMag = magnitudeLimiter.calculate(inputMagnitude);
        } else if (angleDif > 0.85*Math.PI) {
            if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
            // keep currentTranslationDir unchanged
            currentTranslationMag = magnitudeLimiter.calculate(0.0);
            } else {
            currentTranslationDir = wrapAngle(currentTranslationDir + Math.PI);
            currentTranslationMag = magnitudeLimiter.calculate(inputMagnitude);
            }
        } else {
            currentTranslationDir = stepTowardsCircular(currentTranslationDir, inputDirection, directionSlewRate * elapsedTime);
            currentTranslationMag = magnitudeLimiter.calculate(0.0);
        }
        previousTime = currentTime;
        
        xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
        ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
        currentRotation = rotationLimiter.calculate(rotation);

        // Convert the commanded speeds into the correct units for the drivetrain
        
        double xSpeedDelivered = xSpeedCommanded * speedAdjustedMaxSpeed * (RobotContainer.getAlliance().equals(Alliance.Blue) ? 1 : -1);
        double ySpeedDelivered = ySpeedCommanded * speedAdjustedMaxSpeed * (RobotContainer.getAlliance().equals(Alliance.Blue) ? 1 : -1);
        double rotDelivered = currentRotation * maxAngularSpeed;
        SmartDashboard.putNumber("xSpeedCommanded", xSpeedCommanded);
        SmartDashboard.putNumber("ySpeedCommanded", ySpeedCommanded);
        SmartDashboard.putNumber("ground Speed", Math.sqrt(Math.pow(xSpeedDelivered, 2) + Math.pow(ySpeedDelivered, 2)));

        SwerveModuleState[] swerveModuleStates = driveKinematics.toSwerveModuleStates(
            fieldRelative
                // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromRadians(DriverStation.getAlliance() == Alliance.Blue?driveTrain.getGyroAngle() + Math.PI: driveTrain.getGyroAngle() + Math.PI))
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromRadians(RobotContainer.getAlliance().equals(Alliance.Blue) ? driveTrain.getHeading() : driveTrain.getHeading()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        driveTrain.setModuleStates(swerveModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.activityState = ActivityState.IDLE;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param angleA An angle (in radians).
     * @param angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    private static double angleDifference(double angleA, double angleB) {
        double difference = Math.abs(angleA - angleB);
        return difference > Math.PI ? (2 * Math.PI) - difference : difference;
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code current} after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    private static double stepTowardsCircular(double current, double target, double stepsize) {
        current = wrapAngle(current);
        target = wrapAngle(target);

        double stepDirection = Math.signum(target - current);
        double difference = Math.abs(current - target);
        
        if (difference <= stepsize) {
            return target;
        }
        else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (current + 2*Math.PI - target < stepsize || target + 2*Math.PI - current < stepsize) {
                return target;
            }
            else {
                return wrapAngle(current - stepDirection * stepsize); //this will handle wrapping gracefully
            }

        }
        else {
            return current + stepDirection * stepsize;
        }
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    private static double wrapAngle(double angle) {
        double twoPi = 2 * Math.PI;

        if (angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            return 0.0;
        }
        else if (angle > twoPi) {
            return angle - twoPi*Math.floor(angle / twoPi);
        }
        else if (angle < 0.0) {
            return angle + twoPi*(Math.floor((-angle) / twoPi)+1);
        }
        else {
            return angle;
        }
    }
}