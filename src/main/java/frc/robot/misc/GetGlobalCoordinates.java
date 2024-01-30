package frc.robot.misc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveTrain;

public class GetGlobalCoordinates {
    //robotX and robotY is the global coordinates of the robot
    //rotationAngle is the rotation angle of the robot
    public  double pi = Math.PI;
    public  double robotX;
    public  double robotY;
    //assume the y axis is pointing forward and the x axis is pointing to the right
    //the rotationAngle of the robot
    public  double rotationAngle;
    //input x and input y are coordinates reletive to bot
    //x axis point to right, y axis point to font
    public  Translation2d pieceData;
    public  double inputX;
    public  double inputY ;
    public  double xAxisDirection = rotationAngle -pi/2;
    //this is the radius of polar
    public  double targetDistanceToBot;
    public  double targetToRobotAngle;//target angle reletive to the x-axis of the robot
    public  double globalAngleOfTarget;
    //these are the global coordinates of gamepieces
    public double globalX;
    public double globalY;

    public GetGlobalCoordinates(DriveTrain driveTrain, Translation2d pieceData){
        Pose2d currentPose = driveTrain.getPose();
        this.robotX = currentPose.getX();
        this.robotY = currentPose.getY();
        this.rotationAngle = currentPose.getRotation().getRadians();
        this.pieceData = pieceData;

        this.inputX = pieceData.getX();
        this.inputY = pieceData.getY();
        this.targetToRobotAngle = getTargetAngleToRobot(this.inputX,this.inputY);

        this.globalAngleOfTarget = this.rotationAngle + this.targetToRobotAngle;
        System.out.println("Global Angle " + globalAngleOfTarget);
        this.globalX = this.targetDistanceToBot * Math.cos(globalAngleOfTarget)+this.robotX;
        this.globalY = this.targetDistanceToBot * Math.sin(globalAngleOfTarget)+this.robotY;
        System.out.println("Y: " + this.globalY);
    }
    
    //get the target angle reletive to x-axis of robot
    public double getTargetAngleToRobot(double xToRobot, double yToRobot){
        System.out.println(yToRobot + ", " + xToRobot);
        double targetAngle = Math.asin(yToRobot / xToRobot);

        System.out.println(targetAngle * 180/Math.PI);
        return targetAngle;
    }
}