package frc.robot.commands.LEDs;

import java.util.ArrayDeque;

import javax.swing.text.AbstractDocument.LeafElement;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.util.Color;

public class LEDDisplayDrive extends Command implements Constants.Drive {
    DriveTrain driveTrain;
    LED LED;
    GenericHID controller = new GenericHID(0);


    public LEDDisplayDrive(DriveTrain driveTrain, LED LED) 
    {
        this.driveTrain = driveTrain;
        this.LED = LED;
        addRequirements(driveTrain);
        addRequirements(LED);
    }

    @Override
    public void execute() {
        while(driveTrain.getXMovement() > 0 && driveTrain.getXMovement() > driveTrain.getYMovement()){
            LED.setPixel(10, Color.kAqua);
        }
    }

    private double getButton(int id) {
        if (Math.abs(controller.getRawAxis(4)) > 0.1)
            return controller.getRawButton(id) ? 1 : 0;
        return 0;
    }
}
