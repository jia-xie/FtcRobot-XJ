package org.firstinspires.ftc.teamcode.util.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Position;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai.UpdateType.*;
import static org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai.UpdateType.LINEAR;

/**
 * <p> Three Wheel Odometry SubThread Class
 *
 * Wheel Diameter:          2 inch (2 * 2.54 cm)
 * Pulse per Revolution:    4096 (Taobao Encoder)
 *                          8192 (REV Encoder)
 *
 *
 * @Author Jason Xie
 */
public class ThreeWheelOdometryForYuntai implements Runnable{

    public enum UpdateType{
        LINEAR, ARC;
    }

    private final double wheelDiameter = 2;     //in
    private final double ppr = 4096;
    private double mmPerEncoder = ((wheelDiameter * Math.PI) / ppr);
    private double ticksPerDeg;

    private double yRightPulse, yLeftPulse, xPulse = 0;
    private double previousYRightPulse, previousYLeftPulse, previousXPulse, previousAngle = 0;


    private boolean isRunning = true;
    private int sleepTime = 0;

    public static double robotX;
    public static double robotY;
    DcMotor yLeft, yRight, x;

    private double angleOffset;
    private int xOffset;
    private int yLeftOffset;
    private int yRightOffset;
    private UpdateType updateType = LINEAR;
    private int runTime = 0;

    //CONSTRUCTORS
    /**
     * Constructor for main thread
     * @param yLeft         left dead wheel in y coordinate
     * @param yRight        left dead wheel in y coordinate
     * @param X             left dead wheel in y coordinate
     * @param robotXInit    initial x coordinate of robot, in inch
     * @param robotYInit    initial y coordinate of robot, in inch
     */
    public ThreeWheelOdometryForYuntai(DcMotorEx yLeft, DcMotorEx yRight, DcMotorEx X, double ticksPerDeg,
                                       double robotXInit, double robotYInit){
        this.x = X;
        this.yLeft = yLeft;
        this.yRight = yRight;
        this.ticksPerDeg = ticksPerDeg;
        this.robotX = robotXInit;
        this.robotY = robotYInit;
    }

    /**
     * Constructor for MultiThreading
     * @param yLeft         left dead wheel in y coordinate
     * @param yRight        left dead wheel in y coordinate
     * @param X             left dead wheel in y coordinate
     * @param ticksPerDeg
     * @param robotXInit    initial x coordinate of robot, in inch
     * @param robotYInit    initial y coordinate of robot, in inch
     * @param sleepTime     thread running interval, in milliseconds
     */
    public ThreeWheelOdometryForYuntai(DcMotor yLeft, DcMotor yRight, DcMotor X,
                                       double ticksPerDeg,
                                       double robotXInit, double robotYInit,
                                       int sleepTime){
        this.x = X;
        this.yLeft = yLeft;
        this.yRight = yRight;
        this.ticksPerDeg = ticksPerDeg;
        this.robotX = robotXInit;
        this.robotY = robotYInit;
        this.sleepTime = sleepTime;
    }

    public ThreeWheelOdometryForYuntai(DcMotor yLeft, DcMotor yRight, DcMotor X,
                                       double ticksPerDeg,
                                       double robotXInit, double robotYInit,
                                       int sleepTime, UpdateType updateType){
        this.x = X;
        this.yLeft = yLeft;
        this.yRight = yRight;
        this.ticksPerDeg = ticksPerDeg;
        this.robotX = robotXInit;
        this.robotY = robotYInit;
        this.sleepTime = sleepTime;
        this.updateType = updateType;
    }

    public ThreeWheelOdometryForYuntai(HardwareMap hardwareMap,
                                       String yLeftDeviceName, String yRightDeviceName, String xDeviceName,
                                       double ticksPerDeg,
                                       double robotXInit, double robotYInit,
                                       int sleepTime, UpdateType updateType){
        yLeft = hardwareMap.get(DcMotor.class, yLeftDeviceName);
        yRight = hardwareMap.get(DcMotor.class, yRightDeviceName);
        x = hardwareMap.get(DcMotor.class, xDeviceName);
        this.ticksPerDeg = ticksPerDeg;
        this.robotX = robotXInit;
        this.robotY = robotYInit;
        this.sleepTime = sleepTime;
        this.updateType = updateType;
    }
    //PULSE UPDATE METHODS

    /**
     * get the current pulse of the horizontal dead wheel
     * @return the current pulse of the horizontal dead wheel
     */
    public int getXPulse(){
        return x.getCurrentPosition() + xOffset;
    }

    /**
     * get the current pulse of the left vertical dead wheel
     * @return  the current pulse of the left vertical dead wheel
     */
    public int getYLeftPulse(){
        return -yLeft.getCurrentPosition() + yLeftOffset;
    }

    /**
     * get the current pulse of the right vertical dead wheel
     * @return  the current pulse of the right vertical dead wheel
     */
    public int getYRightPulse(){
        return -yRight.getCurrentPosition() + yRightOffset;
    }

    /**
     * record all the pulse in global access variables
     */
    private void updatePulse(){
        xPulse = getXPulse();
        yLeftPulse = getYLeftPulse();
        yRightPulse = getYRightPulse();
    }

    public void resetPulseRecord(){
        xOffset = - getXPulse();
        yLeftOffset = - getYLeftPulse();
        yRightOffset = - getYRightPulse();
    }

    //DISPLACEMENT METHODS
    /**
     * get the robot x displacement increment
     * @return x increment, in inch
     */
    private double getXMove(){
        return (xPulse - previousXPulse) * mmPerEncoder;
    }

    /**
     * get the robot y displacement increment
     * @return  y , in inch
     */
    private double getYMove(){
        return .5 * (
                (yRightPulse - previousYRightPulse) + (yLeftPulse - previousYLeftPulse)
        ) * mmPerEncoder;
    }

    /**
     * return the angle based on current saved pulse record in degree
     * @return the current angle of the robot, in degree
     */
    public double angle(){
        return (yRightPulse - yLeftPulse) / ticksPerDeg + angleOffset;
    }

    /**
     * reset angle when the robot is adjacent to the wall
     */
    public void resetAngle(){
        angleOffset = -angle();
    }

    /**
     * reset the robot position manually
     * @param robotX     x coordinate
     * @param robotY     y coordinate
     */
    public void setCurrentPosition(double robotX, double robotY){
        this.robotX = robotX;
        this.robotY = robotY;
    }

    /**
     * <li>Update the position of the robot, assuming the path og the robot between each two updates
     * is a straight line. This is reasonable when the interval of the update is relatively small.
     *
     * <li>According to the test by Candela, on 2020-2021 season, this method has a accuracy of roughly
     * 4 inch and 5 degree during a match
     *
     * @see #updatePositionByArcPath()
     */
    public void updatePositionByStraightPath(){
        updatePulse();
        double xMovement = getXMove();
        double yMovement = getYMove();
        double angle = angle();

        double sin = sin(toRadians(angle));
        double cos = cos(toRadians(angle));

        robotX += cos * xMovement - sin * yMovement;
        robotY += cos * yMovement + sin * xMovement;

        previousYLeftPulse = yLeftPulse;
        previousYRightPulse = yRightPulse;
        previousXPulse = xPulse;
        previousAngle = angle;
    }

    /**
     * <li>Update the position of the robot, assuming the path of the robot between each
     * two updates is an arc. The radians of the arc is the subtraction of the two records
     * <li>This update method is still under test, the accuracy is yigemi.
     *
     * @see #updatePositionByStraightPath()
     */
    public void updatePositionByArcPath(){
        updatePulse();
        double xMovement = getXMove();
        double yMovement = getYMove();
        double arcLength = Math.sqrt(pow(xMovement,2) + pow(yMovement,2));
        double angle = angle();
        double angleDifferenceInRadians = toRadians(angle - previousAngle);

        double distance;
        if(angleDifferenceInRadians == 0){
            distance = arcLength * 1;
        }else{
            double arcAngleInRadians = angleDifferenceInRadians / 2.0;
            distance = arcLength * (sin(arcAngleInRadians) / (arcAngleInRadians));
        }
        double distanceAngle = atan2(yMovement,xMovement) - toRadians(angle);
        robotX += cos(distanceAngle) * distance;
        robotY += sin(distanceAngle) * distance;

        previousYLeftPulse = yLeftPulse;
        previousYRightPulse = yRightPulse;
        previousXPulse = xPulse;
        previousAngle = angle;
    }

    /**
     * @return x position
     */
    public double getRobotX(){
        return robotX;
    }

    /**
     * @return y position
     */
    public double getRobotY(){
        return robotY;
    }

    public Position getCurrentPosition(){return new Position(getRobotX(),getRobotY(),angle());}

    public int getRuntime(){
        return runTime;
    }
    //MultiThreading Methods

    /**
     * restart the thread is stopped
     */
    public void restart(){
        isRunning = true;
    }

    /**
     * stop the thread manually
     */
    public void stop(){
        isRunning = false;
    }


    @Override
    public void run(){
        switch (updateType) {
            case LINEAR:
                while (isRunning) {
                    updatePositionByStraightPath();
                    runTime++;
                    try {
                        Thread.sleep(sleepTime);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                break;
            case ARC:
                while (isRunning) {
                    updatePositionByArcPath();
                    runTime++;
                    try {
                        Thread.sleep(sleepTime);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                break;
        }
    }
}