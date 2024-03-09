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
    public static SparkLimitSwitch leftSwitch;
    public static SparkLimitSwitch rightSwitch;
    CANSparkMax leftClimbMotor;
    CANSparkMax rightClimbMotor;
    SparkPIDController winchPid;
    SparkAbsoluteEncoder leftWinchEncoder;
    SparkAbsoluteEncoder rightWinchEncoder;
    SparkAbsoluteEncoder.Type winchEncoderType;
    
    /**
     * Initializes the winch
     */
    public Winch()
    {
        leftClimbMotor = new CANSparkMax(leftID, MotorType.kBrushless);
        leftSwitch = leftClimbMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.fromId(leftID));
        leftSwitch.enableLimitSwitch(true);

        rightClimbMotor = new CANSparkMax(rightID, MotorType.kBrushless);
        rightSwitch = rightClimbMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.fromId(rightID));
        rightSwitch.enableLimitSwitch(true);


        SmartDashboard.putNumber("winch/p", winchP);
        SmartDashboard.putNumber("winch/i", winchI);
        SmartDashboard.putNumber("winch/d", winchD);

        winchPid = rightClimbMotor.getPIDController();
        winchPid.setP(winchP);
        winchPid.setI(winchI);
        winchPid.setD(winchD);
        winchPid.setFF(winchFF);

        winchEncoderType = SparkAbsoluteEncoder.Type.kDutyCycle;
        leftWinchEncoder = leftClimbMotor.getAbsoluteEncoder(winchEncoderType);
        rightWinchEncoder = rightClimbMotor.getAbsoluteEncoder(winchEncoderType);

        winchPid.setSmartMotionMaxVelocity(MaxVelocity, slotID);
        winchPid.setSmartMotionMinOutputVelocity(MinVelocity, slotID);
        winchPid.setSmartMotionMaxAccel(MaxAccel, slotID);
        winchPid.setSmartMotionAllowedClosedLoopError(AllowedClosedLoopError, slotID);
        winchPid.setFeedbackDevice(rightWinchEncoder);
    
    }

    /**
     * Extends the winch to a setpoint
     * @param setpoint The setpoint to extend the winch to
     */
    public void setSpeed(double speed)
    {
        leftClimbMotor.set(speed);
        rightClimbMotor.follow(leftClimbMotor, true);
    }


    
    public void getPosition()
    {
        System.out.println("left:" + leftWinchEncoder.getPosition());
        System.out.println("right:" + rightWinchEncoder.getPosition());
    }

    public double getCurrentDifference() {
        SmartDashboard.putNumber("winch/leftCurrent", leftClimbMotor.getOutputCurrent());
        SmartDashboard.putNumber("winch/rightCurrent", rightClimbMotor.getOutputCurrent());
        return leftClimbMotor.getOutputCurrent() - rightClimbMotor.getOutputCurrent();
    }

    /**
     * Extends the winch to a setpoint
     * @param setpoint The setpoint to extend the winch to
     */
    public void setPostion(double setpoint)
    {
       winchPid.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
       rightClimbMotor.follow(leftClimbMotor);
       this.getPosition();
    }



    @Override
    public void periodic()
    {
        winchPid.setP(SmartDashboard.getNumber("winchP", 0));
        winchPid.setI(SmartDashboard.getNumber("winchI", 0));
        winchPid.setD(SmartDashboard.getNumber("winchD", 0));
    }

}
