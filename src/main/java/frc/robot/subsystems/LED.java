package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    int r;
    int g;
    int b;
    int h;
    int tickSpeed;
    int state;
    // state is a variable to determine whether red green and blue should be increasing or decreasing.
    
    @Override
    public void periodic() {
        // switch (RobotContainer.colorTable.getSelected()) {
        //     case "red":
        //     ColorWrapRGB(255, 0, 0);
        //     break;
        //     case "orange":
        //     ColorWrapRGB(255, 165, 0);
        //     break;
        //     case "rainbow":
        //     tick();
        //     break;
        //     default:
        //     ColorWrapRGB(0, 0, 0);
        //     break;
            
        // }
        // led.setData(ledBuffer);
    }

    // public static SendableChooser<String> colorTable = new SendableChooser<>();
    // public static SendableChooser<Integer> tickSpeedChooser = new SendableChooser<>();

    // static {
    //   colorTable.addOption("red", "red");
    //   colorTable.addOption("blue", "blue");
    //   colorTable.addOption("rainbow", "rainbow");

    //   colorTable.setDefaultOption("orange", "orange");
    //   tickSpeedChooser.setDefaultOption("one", 2);
    //   tickSpeedChooser.addOption("one", 1);
    //   tickSpeedChooser.addOption("five", 5);
    //   tickSpeedChooser.addOption("ten", 10);

    //   SmartDashboard.putData(colorTable);
    //   SmartDashboard.putData(tickSpeedChooser);
    // }

    public LED(int length) {
        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setData(ledBuffer);
        
        led.start();
        r = 0;
        g = 0;
        b = 0;
        state = 0;
        h = 0;
    }
    public boolean ColorWrapRGB(int r, int g, int b) {
        Color color = new Color(r,g,b);
        setColor(color);
        return false;
    }

    public boolean ColorWrapHSV(int h, int s, int v) {
        Color color = Color.fromHSV(h,s,v);
        setColor(color);
        return false;                            
    }

    public void setColor(Color color) {
        for(int i=0;i<ledBuffer.getLength(); i++)
            ledBuffer.setLED(i, color);
    }

    public void betterRainbow() {
        if(h == 360)
        {
            h = 0;
        }
        ColorWrapHSV(h, 255, 255);
        h++;
    }

    // public void tick() {
    //     for(int i = 0; i < RobotContainer.tickSpeedChooser.getSelected(); i++)
    //     {
    //         betterRainbow();
    //     }
    // }
}
