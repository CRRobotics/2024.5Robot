package frc.robot.subsystems;
import java.net.CacheRequest;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

/**
 * Simulates the launcher subsystem
 */
public class Shooter extends SubsystemBase implements Constants.Shooter {
    TalonFX leftShooterMotor;
    TalonFX rightShooterMotor;
    VelocityVoltage voltageController;
    Slot0Configs slotConfig;
    CANSparkMax pivotMotor;



    // RelativeEncoder encoder;
    private final AbsoluteEncoder pivotAbsEncoder; //this is the rev-through-bore encoder in abs mode
    PIDController pid;
    SparkLimitSwitch bottomSwitch;
    SparkLimitSwitch topSwitch;

    private static final double horizontalAngle = 0.666;
    private static final double verticalAngle = 0.416;

    public enum Positions {
        HORIZONTAL,
        VERTICAL
    }

    private static double deltaTime = 0.02;
    // Create a PID controller whose setpoint's change is subject to maximum
    // velocity and acceleration constraints.
    private final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(1, 40);
    private final ProfiledPIDController m_controller =
        new ProfiledPIDController(5, 0.0, 0.0, m_constraints, deltaTime);

    private double m_goalAngle = horizontalAngle;     

    public Shooter() {
        leftShooterMotor = new TalonFX(leftShooterMotorID);
        leftShooterMotor.setNeutralMode(NeutralModeValue.Coast);

        rightShooterMotor = new TalonFX(rightShooterMotorID);
        rightShooterMotor.setNeutralMode(NeutralModeValue.Coast);

        
        // voltageController =  new VelocityVoltage(0, 0, false, kF,
        //     0, false, true, true);
        voltageController = new VelocityVoltage(0);

        slotConfig = new Slot0Configs();
        slotConfig.kP = 0;
        slotConfig.kI = 0;
        slotConfig.kD = 0;
        leftShooterMotor.getConfigurator().apply(slotConfig);
        
        pivotMotor = new CANSparkMax(pivotMotorID, MotorType.kBrushless);
        pivotAbsEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        double encoderPositionFactor = (2 * Math.PI); // radians
        double encoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        // pivotAbsEncoder.setPositionConversionFactor(1.0/166.667);
        pivotMotor.getEncoder().setPositionConversionFactor(encoderPositionFactor);
        pivotMotor.getEncoder().setVelocityConversionFactor(encoderVelocityFactor);
    
        
        // pid = new PIDController(0, 0, 0);

        bottomSwitch = pivotMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        bottomSwitch.enableLimitSwitch(true);
        topSwitch = pivotMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        topSwitch.enableLimitSwitch(true);

        SmartDashboard.putNumber("pivot/p", 0);
        SmartDashboard.putNumber("pivot/i", 0);
        SmartDashboard.putNumber("pivot/d", 0);
        SmartDashboard.putNumber("pivot/setpoint", 0);
    }

    /**
     * Runs the shooter at a speed
     * @param setpoint The speed to run the shooter at
     */
    public void setSpeed(double setpoint) {
        voltageController.Slot = 0;
        // leftShooterMotor.setControl(voltageController.withVelocity(setpoint));
        leftShooterMotor.set(setpoint);
        rightShooterMotor.setControl(new Follower(leftShooterMotorID, true));
    }

    // /**
    //  * Aims the launcher to a set point in radians
    //  * @param setPoint The set point to aim the launcher at
    //  */
    // public void aim(double setPoint) {
    //     System.out.println("aiming");
    //     setPoint = SmartDashboard.getNumber("pivot/setpoint", 0);
    //     setPoint /= 2 * Math.PI;
    //     pid = new PIDController(SmartDashboard.getNumber("pivot/p", 0), SmartDashboard.getNumber("pivot/i", 0), SmartDashboard.getNumber("pivot/d", 0));
    //     pivotMotor.set(pid.calculate(encoder.getPosition(), setPoint));
    // }

    @Override
    public void periodic() {
      m_controller.setGoal(m_goalAngle);
      pivotMotor.set(m_controller.calculate(pivotAbsEncoder.getPosition()));
  
  
      SmartDashboard.putNumber("pivot position", pivotAbsEncoder.getPosition());
      SmartDashboard.putNumber("pivot speed", pivotAbsEncoder.getVelocity());
    }

    public void goToAngle(Positions position) {
        switch (position) {
          case HORIZONTAL:
            m_goalAngle = horizontalAngle;
            break;
          case VERTICAL:
            m_goalAngle = verticalAngle;
            break;
        }
      }

    public boolean isPivotAtGoal() {
    return Math.abs(pivotAbsEncoder.getPosition() - m_goalAngle) < 0.03;
    }

    public boolean isPivotHorizontal() {
        return Math.abs(pivotAbsEncoder.getPosition() - horizontalAngle) < 0.03;
      }
    
      public boolean isPivotVertical() {
        return Math.abs(pivotAbsEncoder.getPosition() - verticalAngle) < 0.03;
      }

      public void resetEncoders() {
        pivotMotor.getEncoder().setPosition(0);
      }
}