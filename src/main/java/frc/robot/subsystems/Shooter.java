package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
    Slot0Configs leftSlot;
    Slot0Configs rightSlot;
    VelocityVoltage velocity;

    public Shooter()
    {
        leftShooterMotor = new TalonFX(0);
        rightShooterMotor = new TalonFX(0);
        leftPivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightPivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightConfig = new TalonFXConfiguration();
        leftConfig = new TalonFXConfiguration();
        leftSlot = new Slot0Configs();
        rightSlot = new Slot0Configs();
        leftSlot.kP = 0;
        leftSlot.kI = 0;
        leftSlot.kD = 0;
        rightSlot.kP = 0;
        rightSlot.kI = 0;
        rightSlot.kD = 0;
        pid = new PIDController(kP, kI, kD); // will edit later
        encoderL = leftPivotMotor.getEncoder();
        encoderR = rightPivotMotor.getEncoder();
        velocity = new VelocityVoltage(0);
        velocity.Slot = 0;
    }

    /**
     * Fires the launcher at a set point
     * @param setPoint The set point to fire the launcher at
     */
    public void fire(double setPoint)
    {
        leftShooterMotor.setControl(velocity.withVelocity(setPoint));
        rightShooterMotor.setControl(velocity.withVelocity(setPoint));
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