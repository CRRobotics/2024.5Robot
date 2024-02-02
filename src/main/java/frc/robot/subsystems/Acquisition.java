package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

/**
 * Runs the acquisition
 */
public class Acquisition extends SubsystemBase
{
  CANSparkMax aqMotor;
  /**
   * Initializes the acquisition
   */
  public Acquisition()
  {
    aqMotor = new CANSparkMax(0,CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  /**
   * Intakes cargo
   */
  public void intake(double speed)
  {
    aqMotor.set(speed);
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
  public void reject(double speed)
  {
    aqMotor.set(speed);
  }
}
