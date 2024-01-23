package frc.robot.subsystems;

import com.robotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase
{
    public winch()
    {
        CANSparkMax LeftClimbMotor = new CANSparkMax(0,CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax RightClimbMotor = new CANSparkMax(0,CANSparkMaxLowLevel.MotorType.kBrushless);
        PIDController pid = new PIDController(kP, kI, kD);
    }

    public void extend()
    {
        LeftClimbMotor.set(pid.calculate(encoder.getDistance(), setpoint));
        RightClimbMotor.set(pid.calculate(encoder.getDistance(), setpoint));
    }

    public void retract()
    {
        LeftClimbMotor.set(pid.calculate(encoder.getDistance(), setpoint));
        RightClimbMotor.set(pid.calculate(encoder.getDistance(), setpoint));
    }
    
}
