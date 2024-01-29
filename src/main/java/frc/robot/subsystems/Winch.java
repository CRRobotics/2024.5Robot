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
public class Winch extends SubsystemBase implements Constants.Winch 
{
    DigitalInput toplimitSwitch;
    DigitalInput bottomlimitSwitch;
    CANSparkMax leftClimbMotor;
    CANSparkMax rightClimbMotor;
    
    /**
     * Initializes the winch
     */
    public Winch()
    {
        toplimitSwitch = new DigitalInput(0);
        bottomLimitSwitch = new DigitalInput(1);
        leftClimbMotor = new CANSparkMax(leftID, MotorType.kBrushless);
        rightClimbMotor = new CANSparkMax(rightID, MotorType.kBrushless);
    }

    /**
     * Extends the winch to a setpoint
     * @param setpoint The setpoint to extend the winch to
     */
    public void setSpeed(double speed)
    {
        leftClimbMotor.set();
        rightClimbMotor.set();
        if (speed > 0) 
        {
        if (toplimitSwitch.get()) 
        {
            motor.set(0);
        } else 
        {
            motor.set(speed);
        }
        } 
        else 
        {
        if (bottomlimitSwitch.get()) 
        {
            motor.set(0);
        } else {
            motor.set(speed);
        }
    }
    }
}
