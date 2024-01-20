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


public class LEDDisplayDrive extends Command implements Constants.Drive {
    DriveTrain driveTrain;
   
    LED LED;
    GenericHID controller = new GenericHID(0);


    public LEDDisplayDrive(LED LED) 
    {
        this.LED = LED;
        addRequirements(LED);
    }
    private double getXMovement(){
        return LED.getGyro().getVelocityX();
    }
    private double getYMovement(){
        return LED.getGyro().getVelocityY();
    }

    @Override
    public void execute() {
        while(getXMovement() > 0 && getXMovement() > Math.abs(getYMovement())){
            System.out.println("movingup");
            LED.setPixel(10, Color.kBisque);
        }
        while(getYMovement() > 0 && getYMovement() > Math.abs(getXMovement())){
            System.out.println("movingside");
            LED.setPixel(10, Color.kBisque);
        }
    }

    private double getButton(int id) {
        if (Math.abs(controller.getRawAxis(4)) > 0.1)
            return controller.getRawButton(id) ? 1 : 0;
        return 0;
    }
}
