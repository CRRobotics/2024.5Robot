package frc.robot.commands.LEDs;

import java.util.ArrayDeque;

import javax.swing.text.AbstractDocument.LeafElement;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.util.Color;
import com.kauailabs.navx.frc.AHRS;

public class LEDDisplayDrive extends Command implements Constants.Drive {
    DriveTrain driveTrain;
    private final AHRS gyro;
    LED LED;
    GenericHID controller = new GenericHID(0);


    public LEDDisplayDrive(LED LED) 
    {
        this.LED = LED;
        gyro = new AHRS(SPI.Port.kMXP);
        addRequirements(LED);
    }
    private double getXMovement(){
        return gyro.getVelocityX();
    }
    private double getYMovement(){
        return gyro.getVelocityY();
    }

    @Override
    public void execute() {
        while(getXMovement() > 0 && getXMovement() > Math.abs(getYMovement())){
            LED.setPixel(10, Color.kBisque);
        }
    }

    private double getButton(int id) {
        if (Math.abs(controller.getRawAxis(4)) > 0.1)
            return controller.getRawButton(id) ? 1 : 0;
        return 0;
    }
}
