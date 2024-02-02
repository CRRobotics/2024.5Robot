package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Winch extends SubsystemBase
{
    CANSparkMax LeftClimbMotor;
    CANSparkMax RightClimbMotor;
    PIDController pid;
    Encoder encoder;

    public Winch()
    {
        LeftClimbMotor = new CANSparkMax(0,CANSparkMaxLowLevel.MotorType.kBrushless);
        RightClimbMotor = new CANSparkMax(0,CANSparkMaxLowLevel.MotorType.kBrushless);
        pid = new PIDController(Constants.Winch.kP, Constants.Winch.kI, Constants.Winch.kD);
    }

    public void extend(Double setpoint)
    {
        LeftClimbMotor.set(pid.calculate(encoder.getDistance(), setpoint));
        RightClimbMotor.set(pid.calculate(encoder.getDistance(), setpoint));
    }

    public void retract(Double setpoint)
    {
        LeftClimbMotor.set(pid.calculate(encoder.getDistance(), setpoint));
        RightClimbMotor.set(pid.calculate(encoder.getDistance(), setpoint));
    }
    
}
//six rx, ry, rtheta