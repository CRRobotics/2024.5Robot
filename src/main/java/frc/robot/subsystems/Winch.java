package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

/**
 * Simulates the winch
 */
public class Winch extends SubsystemBase implements Constants.Winch 
{
    public SparkLimitSwitch topLeftSwitch;
    public SparkLimitSwitch bottomLeftSwitch;
    public SparkLimitSwitch topRightSwitch;
    public SparkLimitSwitch bottomRightSwitch;
    CANSparkMax leftClimbMotor;
    CANSparkMax rightClimbMotor;
    
    /**
     * Initializes the winch
     */
    public Winch()
    {
        leftClimbMotor = new CANSparkMax(leftID, MotorType.kBrushless);
        topLeftSwitch = leftClimbMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.fromId(leftID));
        topLeftSwitch.enableLimitSwitch(true);
        bottomLeftSwitch = leftClimbMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.fromId(leftID));
        bottomLeftSwitch.enableLimitSwitch(true);

        rightClimbMotor = new CANSparkMax(rightID, MotorType.kBrushless);
        topRightSwitch = rightClimbMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.fromId(rightID));
        topRightSwitch.enableLimitSwitch(true);
        bottomRightSwitch = rightClimbMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.fromId(rightID));
        bottomRightSwitch.enableLimitSwitch(true);
    }

    /**
     * Extends the winch to a setpoint
     * @param setpoint The setpoint to extend the winch to
     */
    public void setSpeed(double speed)
    {
        leftClimbMotor.set(speed);
        rightClimbMotor.set(speed);
    }
}
