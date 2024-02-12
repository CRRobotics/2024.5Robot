package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Indexer extends SubsystemBase implements Constants.Indexer {
    CANSparkMax indexer;

    public Indexer() {
        indexer = new CANSparkMax(indexID, MotorType.kBrushless);
    }

    /**
   * Intakes the cargo (not at a set speed yet)
   */
  public void intake() {
    indexer.set(indexIntakeSpeed);
  }

  /**
   * Stops the acquisition
   */
  public void stop() {
    indexer.set(0);
  }

  /**
   * Ejects the cargo back out (not at a set speed yet)
   */
  public void reject() {
    indexer.set(indexRejectSpeed);
  }

  public void windUp() {
    
  }
}
