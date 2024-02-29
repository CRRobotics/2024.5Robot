package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

/**
 * Simulates the acquisition subsystem
 */
public class Acquisition extends SubsystemBase implements Constants.Acquisition {
  CANSparkMax aqMotor;
  CANSparkMax indexMotor;
  AnalogInput ringSensor;

  public Acquisition() {
    aqMotor = new CANSparkMax(aqID, MotorType.kBrushless);
    indexMotor = new CANSparkMax(indexID, MotorType.kBrushless);
    ringSensor = new AnalogInput(0);
  }

  /**
   * Intakes the cargo (not at a set speed yet)
   */
  public void setSpeeds(double aqSpeed, double indexSpeed) {
    indexMotor.set(aqSpeed);
    aqMotor.set(indexSpeed);
  }

  /**
   * Stops the acquisition
   */
  public void stop() {
    aqMotor.set(0);
    indexMotor.set(0);
  }

  public boolean seesRing(){
    return (ringSensor.getValue() > 100);
  }
}
