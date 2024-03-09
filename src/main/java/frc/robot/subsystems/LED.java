package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
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
        switch (RobotContainer.colorTable.getSelected()) {
            case OFF:
                setColor(new Color(0, 0, 0));
                break;
            case RAINBOW:
                tick();
                break;
            case DRIVING:
                setColor(new Color(5, 255, 25));
                break;
            case AUTO_DRIVING:
                blink(new Color[]{new Color(5, 255, 25), new Color(255, 0, 0)});
                break;
            case AUTO_COLLECTING:
                setColor(new Color(0, 255, 0));
                break;
            case COLLECTED:
                setColor(new Color(255, 255, 0));
                break;
            case AUTO_SHOOTING:
                setColor(new Color(0, 0, 255));
                break;
        }
        led.setData(ledBuffer);
    }

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

    private long blinkTime = 0;
    public void blink(Color[] colors) {
        long elapsedTime = System.currentTimeMillis() - blinkTime;
        if (elapsedTime > (500 * colors.length)) {
            blinkTime = System.currentTimeMillis();
        }
        setColor(colors[(int)(elapsedTime / (500 * colors.length))]);
    }

    // public boolean ColorWrapRGB(int r, int g, int b) {
    //     Color color = new Color(r,g,b);
    //     setColor(color);
    //     return false;
    // }

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

    public void tick() {
        for(int i = 0; i < RobotContainer.tickSpeedChooser.getSelected(); i++)
        {
            betterRainbow();
        }
    }
}
