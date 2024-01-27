package frc.robot.util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final CANSparkMax wheelMotor;
    private final CANSparkMax turnMotor;
  
    private final RelativeEncoder wheelEncoder;
    private final AbsoluteEncoder turnEncoder;
  
    private final SparkPIDController wheelPID;
    private final SparkPIDController turnPID;
  
    private double angularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int wheelMotorID, int turnMotorID, double angularOffset) {
      wheelMotor = new CANSparkMax(wheelMotorID, MotorType.kBrushless);
      turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
  
      wheelMotor.restoreFactoryDefaults();
      turnMotor.restoreFactoryDefaults();
  
      // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
      wheelEncoder = wheelMotor.getEncoder();
      turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
      wheelPID = wheelMotor.getPIDController();
      turnPID = turnMotor.getPIDController();
      wheelPID.setFeedbackDevice(wheelEncoder);
      turnPID.setFeedbackDevice(turnEncoder);
  
      // Apply position and velocity conversion factors for the driving encoder. The
      // native units for position and velocity are rotations and RPM, respectively,
      // but we want meters and meters per second to use with WPILib's swerve APIs.
      wheelEncoder.setPositionConversionFactor(Constants.SwerveModule.wheelEncoderPositionConversion);
      wheelEncoder.setVelocityConversionFactor(Constants.SwerveModule.wheelEncoderVelocityConversion);
  
      // Apply position and velocity conversion factors for the turning encoder. We
      // want these in radians and radians per second to use with WPILib's swerve
      // APIs.
      turnEncoder.setPositionConversionFactor(Constants.SwerveModule.turnEncoderPositionConversion);
      turnEncoder.setVelocityConversionFactor(Constants.SwerveModule.turnEncoderVelocityConversion);
  
      // Invert the turning encoder, since the output shaft rotates in the opposite direction of
      // the steering motor in the MAXSwerve Module.
      turnEncoder.setInverted(Constants.SwerveModule.turnInverted);
  
      // Enable PID wrap around for the turning motor. This will allow the PID
      // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
      // to 10 degrees will go through 0 rather than the other direction which is a
      // longer route.
      turnPID.setPositionPIDWrappingEnabled(true);
      turnPID.setPositionPIDWrappingMinInput(Constants.SwerveModule.turnEncoderPositionPIDMinInput);
      turnPID.setPositionPIDWrappingMaxInput(Constants.SwerveModule.turnEncoderPositionPIDMaxInput);
  
      // Set the PID gains for the driving motor. Note these are example gains, and you
      // may need to tune them for your own robot!
      wheelPID.setP(Constants.SwerveModule.wheelP);
      wheelPID.setI(Constants.SwerveModule.wheelI);
      wheelPID.setD(Constants.SwerveModule.wheelD);
      wheelPID.setFF(Constants.SwerveModule.wheelFF);
      wheelPID.setOutputRange(Constants.SwerveModule.wheelOutputMin, Constants.SwerveModule.wheelOutputMax);
  
      // Set the PID gains for the turning motor. Note these are example gains, and you
      // may need to tune them for your own robot!
      turnPID.setP(Constants.SwerveModule.turnP);
      turnPID.setI(Constants.SwerveModule.turnI);
      turnPID.setD(Constants.SwerveModule.turnD);
      turnPID.setFF(Constants.SwerveModule.turnFF);
      turnPID.setOutputRange(Constants.SwerveModule.turnOutputMin, Constants.SwerveModule.turnOutputMax);
  
      wheelMotor.setIdleMode(Constants.SwerveModule.wheelIdleMode);
      turnMotor.setIdleMode(Constants.SwerveModule.turnIdleMode);
      wheelMotor.setSmartCurrentLimit(Constants.SwerveModule.wheelCurrentLimit);
      turnMotor.setSmartCurrentLimit(Constants.SwerveModule.turnCurrentLimit);
  
      // Save the SPARK MAX configurations. If a SPARK MAX browns out during
      // operation, it will maintain the above configurations.
      wheelMotor.burnFlash();
      turnMotor.burnFlash();
  
      this.angularOffset = angularOffset;
      desiredState.angle = new Rotation2d(turnEncoder.getPosition());
      wheelEncoder.setPosition(0);
    }
  
    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
      // Apply chassis angular offset to the encoder position to get the position
      // relative to the chassis.
      return new SwerveModuleState(wheelEncoder.getVelocity(),
          new Rotation2d(turnEncoder.getPosition() - angularOffset));
    }
  
    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
      // Apply chassis angular offset to the encoder position to get the position
      // relative to the chassis.
      SmartDashboard.putNumber("odometry pose", new SwerveModulePosition(
        wheelEncoder.getPosition(),
        new Rotation2d(turnEncoder.getPosition() - angularOffset)).distanceMeters);
        
      return new SwerveModulePosition(
          wheelEncoder.getPosition(),
          new Rotation2d(turnEncoder.getPosition() - angularOffset));
    }
  
    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
      
      // Apply chassis angular offset to the desired state.
      SwerveModuleState correctedDesiredState = new SwerveModuleState();
      correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
      correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angularOffset));
  
      // Optimize the reference state to avoid spinning further than 90 degrees.
      SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
          new Rotation2d(turnEncoder.getPosition()));
      // SwerveModuleState optimizedDesiredState = correctedDesiredState;
  
      // Command driving and turning SPARKS MAX towards their respective setpoints.
      // SmartDashboard.putNumber("speedMetersPerSecond (allegedly)", wheelEncoder.getVelocityConversion Factor());
      wheelPID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
      turnPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
      this.desiredState = desiredState;
    }
  
    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
      wheelEncoder.setPosition(0);
    }
}
