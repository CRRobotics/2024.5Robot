package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

/**
 * Simulates the acquisition subsystem
 */
public class Intake extends SubsystemBase implements Constants.Acquisition {
  CANSparkMax aqMotor;

  public Intake() {
    aqMotor = new CANSparkMax(aqID, MotorType.kBrushless);
  }

  /**
   * Intakes the cargo (not at a set speed yet)
   */
  public void collect() {
    aqMotor.set(aqIntakeSpeed);
  }

  /**
   * Stops the acquisition
   */
  public void stop() {
    aqMotor.set(0);
  }

  /**
   * Ejects the cargo back out (not at a set speed yet)
   */
  public void reject() {
    aqMotor.set(aqRejectSpeed);
  }
}
