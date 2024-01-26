package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simulates the launcher subsystem
 */
public class Shooter extends SubsystemBase
{
    TalonFX leftShooterMotor;
    TalonFX rightShooterMotor;
    CANSparkMax leftPivotMotor;
    CANSparkMax rightPivotMotor;
    TalonFXConfiguration rightConfig;
    TalonFXConfiguration leftConfig;
    PIDController pid;
    RelativeEncoder encoderL;
    RelativeEncoder encoderR;

    public Shooter()
    {
        leftShooterMotor = new TalonFX(0);
        rightShooterMotor = new TalonFX(0);
        leftPivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightPivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightConfig = new TalonFXConfiguration();
        leftConfig = new TalonFXConfiguration();
        leftShooterMotor.config_kP(0,0);
        leftShooterMotor.config_kI(0,0);
        leftShooterMotor.config_kD(0,0);
        rightShooterMotor.config_kP(0,0);
        rightShooterMotor.config_kI(0,0);
        rightShooterMotor.config_kD(0,0);
        pid = new PIDController(kP, kI, kD); // will edit later
        encoderL = leftPivotMotor.getEncoder();
        encoderR = rightPivotMotor.getEncoder();
    }

    /**
     * Fires the launcher at a set point
     * @param setPoint The set point to fire the launcher at
     */
    public void fire(double setPoint)
    {
        leftShooterMotor.set(TalonFXControlMode.Velocity, setPoint);
        rightShooterMotor.set(TalonFXControlMode.Velocity, setPoint);
    }

    /**
     * Aims the launcher to a set point
     * @param setPoint The set point to aim the launcher at
     */
    public void aim(double setPoint)
    {
        leftPivotMotor.set(pid.calculate(encoderL.getPosition(), setPoint));
        rightPivotMotor.set(pid.calculate(encoderR.getPosition(), setPoint));
    }
}