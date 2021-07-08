package org.firstinspires.ftc.teamcode.util.robotAssembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;
import org.jetbrains.annotations.NotNull;

import static org.firstinspires.ftc.teamcode.util.MathFunction.getOptimizedAngle;

/**
 * DriveTrain Class is used to describe the drive train of the FTC robot with Mecanum wheels and
 * AndyMark NeveRest 20 orbital gear (the actual ratio is 19.2:1)
 *
 * Particular data info:
 * Encoder on NeveRest 20 Gear: 537.6 per revolution
 * diameter of the wheel:       49.458 (mm)
 * displacement per pulse:      0.022757441473741563 (inch)
 *                              0.5780390134330357 (mm)
 *
 * Robot dimension:     18*18*18    (inch)
 * Tape thickness:      2           (inch)
 * Tile dimension:      22.75       (inch) (purely inside of the Tile)
 * Filed dimension:     14.5*14.5   (inch)
 *                      142.5*142.5 (inch)
 *
 * Position Dimension:  133.5*133.5 (inch)
 *
 * @Author
 */
public abstract class DriveTrain {


    public DcMotorEx motorLF, motorRF, motorLB, motorRB = null;
    public int speedIndex = 1000;
    public double powerIndex = 0.8;

    /**
     * <p>Function</p>Initialize the motors of drive train.
     * The functions will assign the corresponding motors to the REV control hub or expansion hub
     * to the <code>DcMotorEx</code> instance, reverse the direction of the motor on the right,
     * reset the encoder record of all the motor actuator.
     *
     * @param hardwareMap   hardwareMap that holds the actuators of the robot
     *
     * @see #initDriveTrain(HardwareMap, int)
     * @see #initDriveTrain(HardwareMap, double)
     */
    public void initDriveTrain(@NotNull HardwareMap hardwareMap){
        motorLF = hardwareMap.get(DcMotorEx.class, "LF");
        motorRF = hardwareMap.get(DcMotorEx.class, "RF");
        motorLB = hardwareMap.get(DcMotorEx.class, "LB");
        motorRB = hardwareMap.get(DcMotorEx.class, "RB");

        motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);

        motorLF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * <p>Function</p>Initialize the motors of drive train.
     * The functions will assign the corresponding motors to the REV control hub or expansion hub
     * to the <code>DcMotorEx</code> instance, reverse the direction of the motor on the right,
     * reset the encoder record of all the motor actuator.
     *
     * @param hardwareMap   hardwareMap that holds the actuators of the robot
     * @param speedIndex    the multiplier used in <code>move</code> to limit the max speed of the robot
     *
     * @see #move(double, double, double)
     */
    public void initDriveTrain(@NotNull HardwareMap hardwareMap, int speedIndex) {
        initDriveTrain(hardwareMap);
        this.speedIndex = speedIndex;
    }

    /**
     * <p>Function</p>Initialize the motors of drive train.
     * The functions will assign the corresponding motors to the REV control hub or expansion hub
     * to the <code>DcMotorEx</code> instance, reverse the direction of the motor on the right,
     * reset the encoder record of all the motor actuator.
     *
     * @param hardwareMap
     * @param powerIndex    the multiplier used in <code>movePower</code> to limit the max
     *                      speed of the robot.
     *
     * @see #movePower(double, double, double)
     */
    public void initDriveTrain(@NotNull HardwareMap hardwareMap, double powerIndex){
        initDriveTrain(hardwareMap);

    }

    /**
     * Move the robot, controlling the motors open-looped.
     * @param x vertical linear speed
     * @param y horizontal linear speed
     * @param w angular speed
     *
     * @see #move(double, double, double)
     */
    public abstract void movePower(double x, double y, double w);


    /**
     * Move the robot according to linear and angular speed
     * @param x vertical linear speed
     * @param y horizontal linear speed
     * @param w angular speed
     *
     * @see #movePower(double, double, double)
     */
    public abstract void move(double x, double y, double w);

    /**
     * Move the robot corresponding to the absolute coordinate system of the playing field
     * @param gamepad
     * @param angularVelocity
     * @param odometry
     *
     * @see #move(double, double, double)
     * @see #movePower(double, double, double)
     */
    public abstract void moveAbsolutely(Gamepad gamepad, double angularVelocity, ThreeWheelOdometryForYuntai odometry);

    /**
     * Force the robot to stop
     *
     * @see #move(double, double, double)
     * @see #movePower(double, double, double)
     */
    public abstract void stop();


    /**
     * Run to the target position, ends in the target position and absolute angle
     *
     * @param currentPosition  current position of the robot
     * @param target       target position that holds the coordinate and the angle in degree
     * @param linearIndex  the speed index of linear movement
     * @param angularIndex the speed index of turing movement
     */
    public abstract void runToPosition(Position currentPosition, Position target, double linearIndex, double angularIndex);
}