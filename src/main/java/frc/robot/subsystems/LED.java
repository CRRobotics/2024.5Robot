package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
    private AddressableLED led;
    private static AddressableLEDBuffer ledBuffer;
    private boolean blinkState;

    private int h;
    private int partitions;
    private int edgeWidth;
    // state is a variable to determine whether red green and blue should be increasing or decreasing.

    public LED(int length, int partitions, int edgeWidth) {
        this.partitions = partitions;
        this.edgeWidth = edgeWidth;
        led = new AddressableLED(8);
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setData(ledBuffer);
        
        led.start();
        h = 0;

        blinkState = false;
    }

    @Override
    public void periodic() {
        // Color color;
        // Color autoColor = new Color(255, 0, 0);
        // Color offColor = new Color(0, 0, 0);
        
        // tickBlink();
        // switch (RobotContainer.activityState) {
        //     case IDLE:
        //         color = getRainbowColor();
        //         break;
        //     case DRIVING:
        //         color = new Color(5, 255, 25);
        //         break;
        //     case COLLECTING:
        //         color = Color.fromHSV(18, 82, 50); // HSV Color for note, but with a lower value
        //         break;
        //     case CENTERING:
        //         color = new Color(222, 240, 22);
        //         break;
        //     case HAS_NOTE:
        //         color = new Color(255, 108, 45); // Approx the color of a note
        //         break;
        //     case SHOOTING:
        //         color = new Color(5, 72, 255);
        //         break;
        //     case CLIMBING:
        //         color = new Color(255, 0, 204);
        //         break;
        //     default:
        //         color = getRainbowColor();
        //         System.out.println("(LEDs) Unknown activity state");
        //         break;
        // }

        // switch (RobotContainer.controlState) {
        //     case AUTO:
        //         blink(color, offColor);
        //         setEdgesColor(autoColor);
        //         break;
        //     case PATHING:
        //         setColor(color);
        //         blinkEdges(autoColor, null);
        //         break;
        //     case MANUAL:
        //         setColor(color);
        //         break;
        //     default:
        //         System.out.println("(LEDs) Unknown control state");
        //         break;
        // }
        led.setData(ledBuffer);
    }

    // private Color getRainbowColor() {
    //     if(h >= 360)
    //     {
    //         h = 0;
    //     }
    //     h += RobotContainer.rainbowTickSpeedChooser.getSelected();
    //     return Color.fromHSV(h, 255, 255);
    // }
    
    // private int blinkTick;
    // private void tickBlink() {
    //     if (blinkTick > RobotContainer.blinkTickSpeedChooser.getSelected()) {
    //         blinkState = !blinkState;
    //         blinkTick = 0;
    //     } else {blinkTick++;}
    // }

    // /** NULL-Safe */
    // private void blinkEdges(Color colorA, Color colorB) {
    //     if (blinkState && colorA != null) {
    //         setEdgesColor(colorA);
    //     } else if (colorB != null) {
    //         setEdgesColor(colorB);
    //     }
    // }

    // /** NULL-Safe */
    // private void blink(Color colorA, Color colorB) {
    //     if (blinkState && colorA != null) {
    //         setColor(colorA);
    //     } else if (colorB != null) {
    //         setColor(colorB);
    //     }
    // }
    
    // private void setEdgesColor(Color color) {
    //     int partitionSize = ledBuffer.getLength() / partitions;
    //     for(int i = 0; i < partitions; i++) {
    //         int startID = partitionSize * i;
    //         for(int id = startID; id < startID + edgeWidth; id++)
    //         ledBuffer.setLED(id, color);
    //         for(int id = startID + partitionSize - edgeWidth; id < startID + partitionSize; id++)
    //         ledBuffer.setLED(id, color);
    //     }
    // }
    
    public static void setColor(Color color) {
        for(int i=0;i<ledBuffer.getLength(); i++)
        ledBuffer.setLED(i, color);
    }
}
