package frc.robot.subsystems;

import com.robotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase
{
    LeftClimbMotor encoderL;
    RightClimbMotor encoderR;
    public winch()
    {
        CANSparkMax LeftClimbMotor = new CANSparkMax(0,CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax RightClimbMotor = new CANSparkMax(0,CANSparkMaxLowLevel.MotorType.kBrushless);
        PIDController pid = new PIDController(kP, kI, kD);
        encoderL = LeftClimbMotor.encoder;
        encoderR = RightClimbMotor.encoder;
    }

    public void extend(double setPoint)
    {
        LeftClimbMotor.set(pid.calculate(encoderL.getDistance(), setPoint));
        RightClimbMotor.set(pid.calculate(encoderR.getDistance(), setPoint));
    }

    public void retract(double setPoint)
    {
        LeftClimbMotor.set(pid.calculate(encoderL.getDistance(), setPoint));
        RightClimbMotor.set(pid.calculate(encoderR.getDistance(), setPoint));
    }
    
}
