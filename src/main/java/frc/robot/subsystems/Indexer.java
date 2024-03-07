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
      indexerMotor.setVoltage(12);
  }

  /**
  * Intakes the note
  */
  public boolean intake() {
    if (!seesRing()){
      indexerMotor.set(indexIntakeSpeed);
      System.out.println("running intake motors, speed: "+ indexIntakeSpeed);
      return false;
    } else {
        indexerMotor.set(0);
        return true;
    }
  }

  /**
   * Sets index motor speed in percentage
   * @param speed
   */
  public void setSpeed(double speed) {
    indexerMotor.set(speed);
  }


  /**
   * Stops the acquisition
   */
  public void stop() {
    indexerMotor.set(0);
  }

  /**
   * Ejects the note back out
   */
  public void reject() {
    indexerMotor.set(indexRejectSpeed);
  }

  /**
   * Tells information about the note
   * @return if the robot can see a note
   */
  public boolean seesRing(){
    return (ringSensor.getValue() > 100);
  }
}
