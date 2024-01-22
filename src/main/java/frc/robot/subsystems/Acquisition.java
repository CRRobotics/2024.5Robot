package frc.robot.subsystems;

import com.robotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Acquisition extends SubsystemBase
{
  CANSparkMax acquisitionMotor = new CANSparkMax(0,CANSparkMaxLowLevel.MotorType.kBrushless);
}
