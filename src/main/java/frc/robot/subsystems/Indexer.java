package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

public class Indexer extends SubsystemBase implements Constants.Indexer {
    CANSparkMax indexerMotor;
    AnalogInput ringSensor;
    

    public Indexer() {
        indexerMotor = new CANSparkMax(indexID, MotorType.kBrushless);
        ringSensor = new AnalogInput(0);
    }

    /**
   * Intakes the cargo (not at a set speed yet)
   */
  public void intake() {
    if (!seesRing()){
        indexerMotor.set(indexIntakeSpeed);
    } else {
        indexerMotor.set(0);
    }
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

  public boolean seesRing(){
    return (ringSensor.getValue() > 100);
  }
}
