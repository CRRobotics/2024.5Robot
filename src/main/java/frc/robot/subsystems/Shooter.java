package frc.robot.subsystems;
import java.net.CacheRequest;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

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
    RelativeEncoder encoder;
    PIDController pid;
    SparkLimitSwitch bottomSwitch;
    SparkLimitSwitch topSwitch;

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
        
        pivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        encoder = pivotMotor.getEncoder();
        encoder.setPositionConversionFactor(1.0/100);
        pid = new PIDController(0, 0, 0);

        // bottomSwitch = pivotMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        // bottomSwitch.enableLimitSwitch(true);
        // topSwitch = pivotMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        // topSwitch.enableLimitSwitch(true);

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
        leftShooterMotor.setControl(voltageController.withVelocity(setpoint));
        rightShooterMotor.setControl(new Follower(leftShooterMotorID, true));
    }

    /**
     * Aims the launcher to a set point
     * @param setPoint The set point to aim the launcher at
     */
    public void aim(double setPoint) { // is this radians
        System.out.println("aiming");
        setPoint = SmartDashboard.getNumber("pivot/setpoint", 0);
        pid = new PIDController(SmartDashboard.getNumber("pivot/p", 0), SmartDashboard.getNumber("pivot/i", 0), SmartDashboard.getNumber("pivot/d", 0));
        pivotMotor.set(pid.calculate(encoder.getPosition(), setPoint));
    }
}