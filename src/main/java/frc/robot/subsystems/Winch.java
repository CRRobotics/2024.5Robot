package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;

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
    SparkPIDController winchPid;
    SparkAbsoluteEncoder winchEncoder;
    SparkAbsoluteEncoder.Type winchEncoderType;
    
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


        SmartDashboard.putNumber("winch/p", winchP);
        SmartDashboard.putNumber("winch/i", winchI);
        SmartDashboard.putNumber("winch/d", winchD);

        winchPid = rightClimbMotor.getPIDController();
        winchPid.setP(winchP);
        winchPid.setI(winchI);
        winchPid.setD(winchD);
        winchPid.setFF(winchFF);

        winchEncoderType = SparkAbsoluteEncoder.Type.kDutyCycle;
        winchEncoder = leftClimbMotor.getAbsoluteEncoder(winchEncoderType);

        winchPid.setSmartMotionMaxVelocity(MaxVelocity, slotID);
        winchPid.setSmartMotionMinOutputVelocity(MinVelocity, slotID);
        winchPid.setSmartMotionMaxAccel(MaxAccel, slotID);
        winchPid.setSmartMotionAllowedClosedLoopError(AllowedClosedLoopError, slotID);
        winchPid.setFeedbackDevice(winchEncoder);
    
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

    /**
     * Extends the winch to a setpoint
     * @param setpoint The setpoint to extend the winch to
     */
    public void setSpeed2(double speed)
    {
       winchPid.setReference(speed, CANSparkMax.ControlType.kSmartMotion);
       rightClimbMotor.follow(leftClimbMotor);
    }
}
