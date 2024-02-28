package frc.robot.util;

import java.util.TreeMap;

import frc.robot.Robot;
import frc.robot.util.AngleSpeed;


class AngleFromDistance {
    public static double firstNDeriv(double distance)
    {
        double floor = Robot.treeMap.floorKey(distance);
        double ceiling = Robot.treeMap.ceilingKey(distance);
        double dX = ceiling - floor;
        double dTheta = Robot.treeMap.get(ceiling).getAngle() - Robot.treeMap.get(ceiling).getAngle();
        
        return dTheta/dX;
    }
    /**
    public static int secondNDeriv(double distance)
    {
        boolean floorFloor = false;
        boolean ceilingCeiling = true;
        if(Robot.treeMap.floorKey(Robot.treeMap.floorKey(distance) - .0001) != null)
            floorFloor = true;
        if(Robot.treeMap.ceilingKey(Robot.treeMap.ceilingKey(distance) + .0001) != null)
            ceilingCeiling = true;
        if(Robot.treeMap.get(Robot.treeMap.floorKey(distance)) + Robot.treeMap.get(Robot.treeMap.ceilingKey(distance)))
        

    }
    */

    
    public static AngleSpeed interpolateAngleLinear(double distance)
    {
        if(Robot.treeMap.firstKey() > distance)
            return Robot.treeMap.firstEntry().getValue();
        else if(Robot.treeMap.lastKey() < distance)
            return Robot.treeMap.lastEntry().getValue();

        double dThetaOverDX = firstNDeriv(distance);
        double x = distance - Robot.treeMap.floorKey(distance);
        double theta = x * dThetaOverDX + Robot.treeMap.get(Robot.treeMap.floorKey(distance)).getAngle();
        // dtheta/dx * x + initial theta gives the theta value between floor and ceiling at distance (linear interpolation)
        return new AngleSpeed(theta, interpolateSpeedLinear(distance));
    }
    public static double interpolateSpeedLinear(double distance)
    {
        return 0;
        //unfinished method stub   
    }
    
}