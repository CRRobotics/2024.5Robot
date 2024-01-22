package frc.robot.subsystems;

import com.robotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Aquisition extends SubsystemBase
{
  CANSparkMax aquisitionMotor = new CANSparkMax(0,CANSparkMaxLowLevel.MotorType.kBrushless);
}
