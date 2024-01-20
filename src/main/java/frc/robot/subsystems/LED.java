package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import com.kauailabs.navx.frc.AHRS;

public class LED extends SubsystemBase {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private final AHRS gyro;

    int r;
    int g;
    int b;
    int h;
    int tickSpeed;
    int state;
    // state is a variable to determine whether red green and blue should be increasing or decreasing.

    public LED(int length){
        gyro = new AHRS(SPI.Port.kMXP);
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
    public AHRS getGyro(){
        return gyro;
    }
    public boolean ColorWrapRGB(int r, int g, int b)
    {
        Color color = new Color(r,g,b);
        setColor(color);
        return false;
    }
    public boolean ColorWrapHSV(int h, int s, int v)
    {
        Color color = Color.fromHSV(h,s,v);
        setColor(color);
        return false;                            
    }
    public void setColor(Color color)
    {
        for(int i=0;i<ledBuffer.getLength(); i++)
            ledBuffer.setLED(i, color);
    }
    public void setPixel(int p, Color color){
        ledBuffer.setLED(p, color);
    }
    /**
     * // DEPRECATED!!!
     * 
    public void rainbow()
    {
        if(state == 0)
        {
            r = 255;
            state = 1;
        }
        else if(state == 1 && g != 255)
        {
            g++;
        }
        else if(state == 1 && g== 255)
        {
            state = 2;
        }
        else if (state == 2 && r > 0)
        {
            r--;
        }
        else if (state == 2 && r == 0)
        {
            state = 3;
        }
        else if(state == 3 && b < 255)
        {
            b++;
        }
         else if(state == 3 && b == 255)
        {
            state = 4;
        }
         else if(state == 4 && g > 0)
        {
            g--;
        }
         else if(state == 4 && g == 0)
        {
            state = 5;
        }
         else if(state == 5 && r < 255)
        {
            r++;
        }
        else if(state == 5 && r == 255)
        {
            state = 6;
        }
        else if(state == 6 && b > 0)
        {
            b--;
        }
        else 
        state = 0;
        ColorWrapRGB(r, g, b);
        led.setData(ledBuffer);

    }
    */

    public void betterRainbow()
    {
        if(h == 360)
        {
            h = 0;
        }
        ColorWrapHSV(h, 255, 255);
        h++;
    }
    public void tick()
    {
        for(int i = 0; i < RobotContainer.tickSpeedChooser.getSelected(); i++)
        {
            betterRainbow();
        }
    }

//funny
    
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
}
