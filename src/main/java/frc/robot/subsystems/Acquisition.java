package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simulates the acquisition subsystem
 */
public class Acquisition extends SubsystemBase
{
  CANSparkMax aqMotor;

  /**
   * Initializes the acquisition
   */
  public Acquisition()
  {
    aqMotor = new CANSparkMax(0, MotorType.kBrushless);
  }

  /**
   * Intakes the cargo (not at a set speed yet)
   */
  public void intake()
  {
    aqMotor.set(0);
  }

  /**
   * Stops the acquisition
   */
  public void stop()
  {
    aqMotor.set(0);
  }

  /**
   * Ejects the cargo back out (not at a set speed yet)
   */
  public void reject()
  {
    aqMotor.set(0);
  }
}
