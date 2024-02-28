package frc.robot.util;

import java.util.TreeMap;

import frc.robot.Robot;
import frc.robot.util.AngleSpeed;


class AngleFromDistance {
    double distance;
    

    public AngleFromDistance(double distance)
    {
        this.distance = distance;

    }

    public int firstNDeriv()
    {
        double floor = Robot.treeMap.floorKey(distance);
        double ceiling = Robot.treeMap.ceilingKey(distance);
    }
    public int secondNDeriv()
    {

    }
    
    public double interpolateAngle()
    
}