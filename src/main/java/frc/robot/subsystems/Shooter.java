package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    public Shooter()
    {
        TalonFX leftShooterMotor = new TalonFX(0);
        TalonFX rightShooterMotor = new TalonFX(0);
        rightConfig = new TalonFXConfiguration();
        leftConfig = new TalonFXConfiguration();
        leftShooterMotor.config_kP(0,0);
        leftShooterMotor.config_kI(0,0);
        leftShooterMotor.config_kD(0,0);
        rightShooterMotor.config_kP(0,0);
        rightShooterMotor.config_kI(0,0);
        rightShooterMotor.config_kD(0,0);
    }

    public void fire(double setPoint)
    {
        leftShooterMotor.set(TalonFXControlMode.Velocity, setPoint);
        rightShooterMotor.set(TalonFXControlMode.Velocity, setPoint)
    }
}