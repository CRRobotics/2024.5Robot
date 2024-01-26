package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

/**
 * Simulates the winch
 */
public class Winch extends SubsystemBase implements Constants.Winch {
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
        leftClimbMotor = new CANSparkMax(leftID, MotorType.kBrushless);
        rightClimbMotor = new CANSparkMax(rightID, MotorType.kBrushless);
        pid = new PIDController(kP, kI, kD);
        encoderL = leftClimbMotor.getEncoder();
        encoderR = rightClimbMotor.getEncoder();
    }

    /**
     * Extends the winch to a setpoint
     * @param setpoint The setpoint to extend the winch to
     */
    public void setPosition(double setpoint)
    {
        leftClimbMotor.set(pid.calculate(encoderL.getPosition(), setpoint));
        rightClimbMotor.set(pid.calculate(encoderR.getPosition(), setpoint));
    }
}
