package frc.robot.util;

import java.util.TreeMap;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.AngleSpeed;


class AngleFromDistance {
    private static double firstNDeriv(double distance)
    {
        double floor = RobotContainer.treeMap.floorKey(distance);
        double ceiling = RobotContainer.treeMap.ceilingKey(distance);
        double dX = ceiling - floor;
        double dTheta = RobotContainer.treeMap.get(ceiling).getAngle() - RobotContainer.treeMap.get(ceiling).getAngle();
        
        return dTheta/dX;
    }

    /**
    public static int secondNDeriv(double distance)
    {
        boolean floorFloor = false;
        double bottomKey;
        boolean ceilingCeiling = false;
        double highestKey;
        double deltaX;
        double secondDeriv;
        double nDeriv1;
        double nDeriv2;
        double nDeriv3;
        
        if(Robot.treeMap.floorKey(Robot.treeMap.floorKey(distance) - .0001) != null)
        {
            bottomKey = Robot.treeMap.floorKey(Robot.treeMap.floorKey(distance) - .0001);
            floorFloor = true;
            deltaX = distance - bottomKey;
            nDeriv1 = firstNDeriv(Robot.treeMap.floorKey(distance) - .0001);
        }
        if(Robot.treeMap.ceilingKey(Robot.treeMap.ceilingKey(distance) + .0001) != null)
        {
            ceilingCeiling = true;
            nDeriv3 = firstNDeriv(Robot.treeMap.ceilingKey(distance) + .0001);
            highestKey = Robot.treeMap.ceilingKey(Robot.treeMap.ceilingKey(distance) + .0001);
        }
        if(!floorFloor)
        {
            deltaX = Robot.treeMap.floorKey(distance);
        }
        nDeriv2 = firstNDeriv(distance);
        if(floorFloor && ceilingCeiling)
        {
            secondDeriv = (highestKey - bottomKey);
        }
        else if(floorFloor)
        {
            nDeriv1 + nDeriv2 
        }
        else if(ceilingCeiling)
        {}
        else
        {
            System.out.println("THE 2nd DERIVATIVE DOESNT EXIST BETWEEN THESE POINTS");
            return 0;

        } 
        if(Robot.treeMap.get(Robot.treeMap.floorKey(distance)).getAngle() + Robot.treeMap.get(Robot.treeMap.ceilingKey(distance)).getAngle() /)
        
    }
    private static double linearizeSlopes(double deltaX, double distance)
    {
        
        return 0;
    }
    */

    
    public static AngleSpeed interpolateAngleSpeedLinear(double distance)
    {
        if(RobotContainer.treeMap.firstKey() > distance)
            return RobotContainer.treeMap.firstEntry().getValue();
        else if(RobotContainer.treeMap.lastKey() < distance)
            return RobotContainer.treeMap.lastEntry().getValue();

        double dThetaOverDX = firstNDeriv(distance);
        double x = distance - RobotContainer.treeMap.floorKey(distance);
        double theta = x * dThetaOverDX + RobotContainer.treeMap.get(RobotContainer.treeMap.floorKey(distance)).getAngle();
        // dtheta/dx * x + initial theta gives the theta value between floor and ceiling at distance (linear interpolation)
        return new AngleSpeed(theta, interpolateSpeedLinear(distance));
    }
    public static double interpolateSpeedLinear(double distance)
    {
        return 0;
        //unfinished method stub   
    }
    
}