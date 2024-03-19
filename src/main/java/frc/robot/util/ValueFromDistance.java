package frc.robot.util;

import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ValueFromDistance {

    public static TreeMap<Double, AngleSpeed> shootMap = new TreeMap<>();
    static {
        //put values here
        // shootMap.put(0 * 0.3048 + 1.23, new AngleSpeed(4.7, 160));
        // shootMap.put(1 * 0.3048 + 1.23, new AngleSpeed(4.85, 160));
        // shootMap.put(2 * 0.3048 + 1.23, new AngleSpeed(4.95, 160));
        // shootMap.put(3 * 0.3048 + 1.23, new AngleSpeed(5.02, 160));
        // shootMap.put(4 * 0.3048 + 1.23, new AngleSpeed(5.03, 160));
        shootMap.put(0.0, new AngleSpeed(4.79, 160)); //real
        shootMap.put(1.29, new AngleSpeed(4.8, 160));
        shootMap.put(1.75, new AngleSpeed(4.9, 160));
        shootMap.put(2.13, new AngleSpeed(5, 160));
    }

    public static AngleSpeed getAngleSpeed(double distance) {
        if(distance > shootMap.firstKey() && distance < shootMap.lastKey()) {
            SmartDashboard.putNumber("distance from speaker", distance);
            double floorDistance = shootMap.floorKey(distance);
            double ceilingDistance = shootMap.ceilingKey(distance);
            double remainderFrac = (distance % floorDistance) / (ceilingDistance - floorDistance);

            double speed = shootMap.get(floorDistance).getSpeed() + remainderFrac * (Math.abs(shootMap.get(ceilingDistance).getSpeed() - shootMap.get(floorDistance).getSpeed()));
            double angle = shootMap.get(floorDistance).getAngle() + remainderFrac * (Math.abs(shootMap.get(ceilingDistance).getAngle() - shootMap.get(floorDistance).getAngle()));
            System.out.println("WORKING!");
            return new AngleSpeed(angle, speed);
        }
        else return (distance < shootMap.firstKey()) ?
            //THIS WORKS DONT THINK TOO HARD ABOUT IT
            //gets the angle speed of the floor or ceiling key
            new AngleSpeed(shootMap.get(shootMap.firstKey()).getAngle(), shootMap.get(shootMap.firstKey()).getSpeed()):
            new AngleSpeed(shootMap.get(shootMap.lastKey()).getAngle(), shootMap.get(shootMap.lastKey()).getSpeed());
    }

    public static AngleSpeed getAngleSpeedLinearized(double distance) {
        SmartDashboard.putNumber("distance from speaker", distance);
        if (distance > shootMap.firstKey() && distance < shootMap.lastKey()) {
            double floorDistance = shootMap.floorKey(distance); 
            double ceilingDistance = shootMap.ceilingKey(distance);

            double run = ceilingDistance - floorDistance;
            
            double speed = shootMap.get(floorDistance).getSpeed() + (distance - floorDistance) * ((shootMap.get(ceilingDistance).getSpeed() - shootMap.get(floorDistance).getSpeed())/run);
            double angle = shootMap.get(floorDistance).getAngle() + (distance - floorDistance) * ((shootMap.get(ceilingDistance).getAngle() - shootMap.get(floorDistance).getAngle())/run);

            return new AngleSpeed(angle, speed);
        }
        else if (distance < shootMap.firstKey()) 
        {
            return shootMap.get(shootMap.firstKey());
        }
        else if (distance > shootMap.lastKey()) {
            return shootMap.get(shootMap.lastKey());
        }
        else return shootMap.get(shootMap.firstKey());
    }

    public static AngleSpeed getAngleSpeedFloored(double distance) {
        double flooredVal = shootMap.floorKey(distance);
        return new AngleSpeed(shootMap.get(flooredVal).getAngle(),
                shootMap.get(flooredVal).getSpeed());
    }

    public static double getDistanceToTarget(Pose2d current, Translation2d target) {
        return current.getTranslation().getDistance(target);
    }

}
