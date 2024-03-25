package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Constants;

public class Indexer extends SubsystemBase implements Constants.Indexer {
  CANSparkMax indexerMotor;
  AnalogInput ringSensor;
  boolean sawRing;
    

  public Indexer() {
    sawRing = false;
    indexerMotor = new CANSparkMax(indexID, MotorType.kBrushless);
    ringSensor = new AnalogInput(0);
    // indexerMotor.setVoltage(12);
  }

  @Override
  public void periodic() {
    System.out.println(seesRing());
    if (seesRing() && !sawRing) {
      RobotContainer.setLEDs(Color.kYellow);
      sawRing = true;
    } else if (!seesRing() && sawRing) {
      RobotContainer.setLEDs(Color.kRed);
      sawRing = false;
    }
  }

  /**
  * Intakes the note
  @return if the beam is broken
  */
  public boolean intake() {
    if (!seesRing()){
      indexerMotor.set(indexIntakeSpeed);
      System.out.println("running intake motors, speed: "+ indexIntakeSpeed);
      return false;
    } else {
        // RobotContainer.setLEDs(new Color(255, 109, 46));
        RobotContainer.activityState = RobotContainer.ActivityState.HAS_NOTE;
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
    System.out.println(ringSensor.getValue());
    return (ringSensor.getValue() > 100);
  }
}
