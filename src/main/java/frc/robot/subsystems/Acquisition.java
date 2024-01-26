package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

/**
 * Simulates the acquisition subsystem
 */
public class Acquisition extends SubsystemBase implements Constants.Acquisition {
  CANSparkMax aqMotor;
  CANSparkMax leftIndexMotor;
  CANSparkMax rightIndexMotor;

  public Acquisition() {
    aqMotor = new CANSparkMax(aqID, MotorType.kBrushless);
    leftIndexMotor = new CANSparkMax(leftID, MotorType.kBrushless);
    rightIndexMotor = new CANSparkMax(rightID, MotorType.kBrushless);
    rightIndexMotor.setInverted(true);
  }

  /**
   * Intakes the cargo (not at a set speed yet)
   */
  public void intake() {
    aqMotor.set(aqIntakeSpeed);
    leftIndexMotor.set(indexIntakeSpeed);
    rightIndexMotor.set(indexIntakeSpeed);
  }

  /**
   * Stops the acquisition
   */
  public void stop() {
    aqMotor.set(0);
    leftIndexMotor.set(0);
    rightIndexMotor.set(0);
  }

  /**
   * Ejects the cargo back out (not at a set speed yet)
   */
  public void reject() {
    aqMotor.set(aqRejectSpeed);
    leftIndexMotor.set(indexRejectSpeed);
    rightIndexMotor.set(indexRejectSpeed);
  }
}
