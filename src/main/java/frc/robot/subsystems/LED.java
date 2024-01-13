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

    public LED(int length){
        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setData(ledBuffer);
        
        led.start();
        
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

  
    @Override
    public void periodic() {
        switch (RobotContainer.colorTable.getSelected()) {
            case "red":
            ColorWrapRGB(255, 0, 0);
            break;
            case "orange":
            ColorWrapRGB(255, 165, 0);
            break;
            default:
            ColorWrapRGB(0, 0, 0);
            break;
            
        }
        led.setData(ledBuffer);
    }
}
