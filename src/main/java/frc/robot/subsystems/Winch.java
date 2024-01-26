package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simulates the winch
 */
public class Winch extends SubsystemBase
{
    CANSparkMax leftClimbMotor;
    CANSparkMax rightClimbMotor;
    PIDController pid;
    RelativeEncoder encoderL;
    RelativeEncoder encoderR;
    
    /**
     * Initializes the winch
     */
    public Winch()
    {
        leftClimbMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightClimbMotor = new CANSparkMax(0, MotorType.kBrushless);
        pid = new PIDController(kP, kI, kD);
        encoderL = leftClimbMotor.getEncoder();
        encoderR = rightClimbMotor.getEncoder();
    }

    /**
     * Extends the winch to a set point
     * @param setPoint The set point to extend the winch
     */
    public void extend(double setPoint)
    {
        leftClimbMotor.set(pid.calculate(encoderL.getPosition(), setPoint));
        rightClimbMotor.set(pid.calculate(encoderR.getPosition(), setPoint));
    }

    /**
     * Retracts the winch to a set point
     * @param setPoint The set point to retract the winch
     */
    public void retract(double setPoint)
    {
        leftClimbMotor.set(pid.calculate(encoderL.getPosition(), setPoint));
        rightClimbMotor.set(pid.calculate(encoderR.getPosition(), setPoint));
    }
    
}
