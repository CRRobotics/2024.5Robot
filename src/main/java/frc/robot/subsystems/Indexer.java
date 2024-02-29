package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Indexer extends SubsystemBase implements Constants.Indexer {
    CANSparkMax indexerMotor;
    

    public Indexer() {
        indexerMotor = new CANSparkMax(indexID, MotorType.kBrushless);
    }

    /**
   * Intakes the cargo (not at a set speed yet)
   */
  public void intake() {
    indexerMotor.set(indexIntakeSpeed);
    System.out.println("running intake motors, speed: "+ indexIntakeSpeed);
  }

  /**
   * Stops the acquisition
   */
  public void stop() {
    indexerMotor.set(0);
  }

  /**
   * Ejects the cargo back out (not at a set speed yet)
   */
  public void reject() {
    indexerMotor.set(indexRejectSpeed);
  }

  public void windUp() {
    
  }
}
