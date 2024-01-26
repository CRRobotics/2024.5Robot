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
import frc.robot.util.Constants;

/**
 * Simulates the launcher subsystem
 */
public class Shooter extends SubsystemBase implements Constants.Shooter {
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
    VelocityVoltage voltageController;

    public Shooter() {
        leftShooterMotor = new TalonFX(0);
        leftPivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        leftConfig = new TalonFXConfiguration();
        leftSlot = new Slot0Configs();
        leftSlot.kP = 0;
        leftSlot.kI = 0;
        leftSlot.kD = 0;
        encoderL = leftPivotMotor.getEncoder();

        rightShooterMotor = new TalonFX(0);
        rightPivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightConfig = new TalonFXConfiguration();
        rightSlot = new Slot0Configs();
        rightSlot.kP = 0;
        rightSlot.kI = 0;
        rightSlot.kD = 0;
        encoderR = rightPivotMotor.getEncoder();

        //TODO: do we need multiple of these
        voltageController = new VelocityVoltage(null, null, false,
            kF, 0, false, true, true);
            new VelocityVoltage(kF, kF, false, kF, 0, false, false, false)
        // maybe use this too
        // voltageController.clone();
    }

    /**
     * Fires the launcher at a set point
     * @param setPoint The set point to fire the launcher at
     */
    public void fire(double setPoint)
    {
        voltageController.Velocity = setPoint;
        leftShooterMotor.setControl(voltageController);
        rightShooterMotor.setControl(voltageController);
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