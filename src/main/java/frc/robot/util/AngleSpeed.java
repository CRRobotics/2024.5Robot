package frc.robot.util;

/**
 * Wrapper class for angle and speed intended for use in <code>ValueFromDistance.java</code> and <code>SpeakerShot.java</code>
 */
public class AngleSpeed {
    private double angle;
    private double speed;
    
    public AngleSpeed(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
    }

    public double getAngle() {
        return angle;
    }

    public double getSpeed() {
        return speed;
    }
}
