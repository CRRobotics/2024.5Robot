package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

/**
 * Acquisition subsystem
 */
public class Intake extends SubsystemBase implements Constants.Intake {
  CANSparkMax intakeMotor;

  public Intake() {
    intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);
  }

  /**
   * Intakes the note
   */
  public void collect() {
    intakeMotor.set(intakeCollectSpeed);
  }

  /**
   * Stops the acquisition
   */
  public void stop() {
    intakeMotor.set(0);
  }

  /**
   * Ejects the note
   */
  public void reject() {
    intakeMotor.set(intakeRejectSpeed);
  }
}
