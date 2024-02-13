package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    TalonFXConfiguration rightConfig;
    TalonFXConfiguration leftConfig;
    Slot0Configs leftSlot;
    Slot0Configs rightSlot;

    PIDController pid;
    CANSparkMax PivotMotor;
    RelativeEncoder Pivotencoder;
    VelocityVoltage voltageController;

    public Shooter() {
        leftShooterMotor = new TalonFX(0);
        leftConfig = new TalonFXConfiguration();
        leftShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        leftSlot = new Slot0Configs();
        leftSlot.kP = 0;
        leftSlot.kI = 0;
        leftSlot.kD = 0;

        rightShooterMotor = new TalonFX(0);
        rightConfig = new TalonFXConfiguration();
        rightShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        rightSlot = new Slot0Configs();
        rightSlot.kP = 0;
        rightSlot.kI = 0;
        rightSlot.kD = 0;
        
        PivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        Pivotencoder = PivotMotor.getEncoder();



        //TODO: do we need multiple of these
        voltageController =  new VelocityVoltage(0, 0, false, kF,
            0, false, true, true);
        // maybe use this too
        // voltageController.clone();
    }

    /**
     * Runs the shooter at a speed
     * @param setpoint The speed to run the shooter at
     */
    public void setSpeed(double setpoint) {
        voltageController.Velocity = setpoint; // is this rpm
        leftShooterMotor.setControl(voltageController);
        rightShooterMotor.setControl(voltageController);
    }

    /**
     * Aims the launcher to a set point
     * @param setPoint The set point to aim the launcher at
     */
    public void aim(double setPoint) { // is this radians
        PivotMotor.set(pid.calculate(Pivotencoder.getPosition(), setPoint));
    }
}