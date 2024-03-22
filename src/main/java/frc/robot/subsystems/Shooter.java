package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

/**
 * Simulates the launcher subsystem
 */
public class Shooter extends SubsystemBase implements Constants.Shooter {
    private static TalonFX leftShooterMotor;
    private static TalonFX rightShooterMotor;
    VelocityVoltage voltageController;
    Slot0Configs slotConfig;
    Slot0Configs talonSlotConfigs;
    Slot0Configs krakenSlotConfig;
    CANSparkMax pivotMotor;
    AbsoluteEncoder pivotEncoder;
    PIDController pid;
    SparkPIDController sparkPid;
    SparkLimitSwitch bottomSwitch;
    SparkLimitSwitch topSwitch;
    SparkAbsoluteEncoder.Type pivotEncoderType;
    MotionMagicVelocityVoltage talonController;
    public double shooterVelocity;

    public Shooter() {
        // #FLYWHEEL
        // Documentation for TalonFX: https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/index.html
        leftShooterMotor = new TalonFX(leftShooterMotorID);
        leftShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        talonController = new MotionMagicVelocityVoltage(leftShooterMotorID);
        talonController.Acceleration = talonControllerAcceleration;

        rightShooterMotor = new TalonFX(rightShooterMotorID);
        rightShooterMotor.setNeutralMode(NeutralModeValue.Coast);

        /*Quoted Directly from Adam Newhouse
         * Tuning kV and then adding a small amount of kP gain is the best way to tune a flywheel
         *A flywheel is basically an almost perfect linear system where kV will do most of the work   
         *If you are using the kraken encoder as feedback you will be measuring the motor rpm not the flywheel which is fine
         *If you change the gear ratio later, you can proportionally adjust kV and only need minor retuning. 
         *The existing lookup table will most likely continue to work if you adjust it proportionally to the gear ratio change.
         */
        
        // voltageController =  new VelocityVoltage(0, 0, false, kF,
        //     0, false, true, true);
        voltageController = new VelocityVoltage(voltageControllerVelocity);
        krakenSlotConfig = new Slot0Configs();
        krakenSlotConfig.kP = krakenP;
        krakenSlotConfig.kI = krakenI;
        krakenSlotConfig.kD = krakenD;
        krakenSlotConfig.kV = Constants.Shooter.kV;
        

        leftShooterMotor.getConfigurator().apply(krakenSlotConfig);
        
        // AZIMUTH
        pivotMotor = new CANSparkMax(pivotMotorID, MotorType.kBrushless);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoderType = SparkAbsoluteEncoder.Type.kDutyCycle;
        pivotEncoder = pivotMotor.getAbsoluteEncoder(pivotEncoderType);
        // pivotEncoder.setZeroOffset();

        pid = new PIDController(0, 0, 0);
        sparkPid = pivotMotor.getPIDController();

        sparkPid.setP(sparkP);
        sparkPid.setI(sparkI);
        sparkPid.setD(sparkD);
        sparkPid.setFF(sparkFF);

        

        //good deceleration speed
        sparkPid.setSmartMotionMaxVelocity(smartMotionMaxVelocity, slotID);
        sparkPid.setSmartMotionMinOutputVelocity(smartMotionMinVelocity, slotID);
        sparkPid.setSmartMotionMaxAccel(smartMotionMaxAccel, slotID);
        sparkPid.setSmartMotionAllowedClosedLoopError(smartMotionAllowedClosedLoopError, slotID);
        sparkPid.setFeedbackDevice(pivotEncoder);

        // bottomSwitch = pivotMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        // bottomSwitch.enableLimitSwitch(true);
        // topSwitch = pivotMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        // topSwitch.enableLimitSwitch(true);

        SmartDashboard.putNumber("pivot/p", sparkP);
        SmartDashboard.putNumber("pivot/i", sparkI);
        SmartDashboard.putNumber("pivot/d", sparkD);
        SmartDashboard.putNumber("pivot/ff", sparkFF);
        SmartDashboard.putNumber("shooter KP", krakenP);
        SmartDashboard.putNumber("shooter KI", krakenI);
        SmartDashboard.putNumber("shooter KD", krakenD);
        SmartDashboard.putNumber("pivot/setpoint", 0);
        
        SmartDashboard.putNumber("shooter/left current", leftShooterMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shooter/right current", rightShooterMotor.getTorqueCurrent().getValueAsDouble());


        shooterVelocity = 0;
        // SmartDashboard.putNumber("shooter/velocity", shooterVelocity);
        // SmartDashboard.putNumber("shooter/velocity", leftShooterMotor.getVelocity().getValue());
        leftShooterMotor.getConfigurator().apply(krakenSlotConfig);
    }

    public double getSpeed()
    {
        SmartDashboard.putNumber("shooter/left current", leftShooterMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shooter/right current", rightShooterMotor.getTorqueCurrent().getValueAsDouble());

        return leftShooterMotor.getVelocity().getValue() * beltRatio;
    }

    public double getAngle()
    {
        return pivotEncoder.getPosition();
    }

    public boolean isInterfaced()
    {
        return (Math.abs(pivotEncoder.getPosition() - Constants.Shooter.interfaceAngle) < Constants.Shooter.interfaceError); // error is .08 down from .1
    }

    /**
     * Runs the shooter at a speed
     * @param setpoint The speed to run the shooter wheels at in rps
     */
    public void setSpeed(double setpoint) {
        //setpoint is the Rotation Per Second (RPS) of the shooter wheels
        //MotorRPS * MotorPulleyTeethNum = ShooterWheelRPS * ShooterWheelPulleyTeethNum
        //MotorRPM = ShooterWheelRPM * (ShooterWheelPulleyTeethNum/MotorPulleyTeethNum)
        double motorRPS = setpoint * beltRatio;

        voltageController.Slot = 0;
        leftShooterMotor.setControl(talonController.withVelocity(motorRPS));
        rightShooterMotor.setControl(new Follower(leftShooterMotorID, true));
    }

    public void setSpeedPivot(double setpoint) {
        System.out.println(pivotEncoder.getPosition());
        voltageController.Slot = 0;
        // leftShooterMotor.setControl(voltageController.withVelocity(setpoint));
        pivotMotor.set(setpoint);
    }

    /**
     * Aims the launcher to a set point in radians
     * @param setPoint The set point to aim the launcher at
     */
    public void aim(double setPoint) {
        System.out.println("aiming");
        //setPoint = SmartDashboard.getNumber("pivot/setpoint", 0);
        // setPoint = 1.5;
        // setPoint /= 2 * Math.PI;
        //pid = new PIDController(SmartDashboard.getNumber("pivot/p", 0), SmartDashboard.getNumber("pivot/i", 0), SmartDashboard.getNumber("pivot/d", 0));
        // pid = new PIDController(0.1,0,0);
        // pivotMotor.set(pid.calculate(pivotEncoder.getPosition(), setPoint));
        sparkPid.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);

        //pivotMotor.getAbsoluteEncoder.getPosition();
    }

    public void testAim(double setPoint)
    {
        System.out.println(pivotEncoder.getPosition());
        sparkPid.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
    }

    @Override
    public void periodic() {
        // krakenSlotConfig.kP = SmartDashboard.getNumber("shooter KP", 0);
        // krakenSlotConfig.kI = SmartDashboard.getNumber("shooter KI", 0);
        // krakenSlotConfig.kD = SmartDashboard.getNumber("shooter KD", 0);
        
        // sparkPid.setP(SmartDashboard.getNumber("pivot/p", 0));
        // sparkPid.setI(SmartDashboard.getNumber("pivot/i", 0));
        // sparkPid.setD(SmartDashboard.getNumber("pivot/d", 0));
        // sparkPid.setFF(SmartDashboard.getNumber("pivot/ff", 0));

        // leftShooterMotor.getConfigurator().apply(krakenSlotConfig);
        SmartDashboard.putNumber("shooter/velocity", getSpeed());
        SmartDashboard.putNumber("pivot value", pivotEncoder.getPosition());
    }
}