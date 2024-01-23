package frc.robot.subsystems;

import com.robotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Acquisition extends SubsystemBase
{
  public acquisition()
  {
    CANSparkMax aqMotor = new CANSparkMax(0,CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  public void intake()
  {
    aqMotor.set();
  }

  public void stop()
  {
    aqMotor.set(0);
  }

  public void reject()
  {
    aqMotor.set()
  }
}
