package frc.robot.subsystems;

import com.robotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Runs the acquisition
 */
public class Acquisition extends SubsystemBase
{

  /**
   * Initializes the acquisition
   */
  public Acquisition()
  {
    CANSparkMax aqMotor = new CANSparkMax(0,CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  /**
   * Intakes cargo
   */
  public void intake()
  {
    aqMotor.set();
  }

  /**
   * Stops acquisition
   */
  public void stop()
  {
    aqMotor.set(0);
  }

  /**
   * Ejects acquisition out
   */
  public void reject()
  {
    aqMotor.set();
  }
}
