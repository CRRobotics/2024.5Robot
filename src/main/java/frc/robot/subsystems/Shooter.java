package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    RealativeEncoder encoderL;
    RealativeEncoder encoderR;
    public Shooter()
    {
        TalonFX leftShooterMotor = new TalonFX(0);
        TalonFX rightShooterMotor = new TalonFX(0);
        CANSparkMax leftPivotMotor = new TalonFX(0);
        CANSparkMax rightPivotMotor = new TalonFX(0);
        rightConfig = new TalonFXConfiguration();
        leftConfig = new TalonFXConfiguration();
        leftShooterMotor.config_kP(0,0);
        leftShooterMotor.config_kI(0,0);
        leftShooterMotor.config_kD(0,0);
        rightShooterMotor.config_kP(0,0);
        rightShooterMotor.config_kI(0,0);
        rightShooterMotor.config_kD(0,0);
        PIDController pid = new PIDController(kP, kI, kD);
        encoderL = LeftClimbMotor.encoder;
        encoderR = RightClimbMotor.encoder;
    }

    public void fire(double setPoint)
    {
        leftShooterMotor.set(TalonFXControlMode.Velocity, setPoint);
        rightShooterMotor.set(TalonFXControlMode.Velocity, setPoint);
    }

    public void aim(double setPoint)
    {
        leftPivotMotor.set(pid.calculate(encoderL.getDistance(), setPoint));
        rightPivotMotor.set(pid.calculate(encoderR.getDistance(), setPoint));
    }
}