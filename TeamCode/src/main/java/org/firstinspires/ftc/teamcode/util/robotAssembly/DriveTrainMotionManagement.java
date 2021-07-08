package org.firstinspires.ftc.teamcode.util.robotAssembly;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.util.MathFunction.getBig;
import static org.firstinspires.ftc.teamcode.util.MathFunction.getOptimizedAngle;

/**
 * <p>DriveTrainMotionManagement</p>
 * This class stores the control method of drive train. For the convenient purpose,
 * we extends this class to make the control methods more clear to the users and more
 * easily to use.
 *
 * @Author Candela
 */
public class DriveTrainMotionManagement extends DriveTrain {

    /**
     *
     * @param x vertical linear speed
     * @param y horizontal linear speed
     * @param w angular speed
     *
     * @see #move(double, double, double)
     */
    @Override
    public void movePower(double x, double y, double w) {
        double speed_lf = y + x + w;
        double speed_lb = y - x + w;
        double speed_rf = y - x - w;
        double speed_rb = y + x - w;

        double max = getBig(getBig(getBig(Math.abs(speed_lf),
                Math.abs(speed_rf)), Math.abs(speed_lb)), Math.abs(speed_rb));
        if (max > 1) {
            speed_lf *= 1 / max;
            speed_rf *= 1 / max;
            speed_lb *= 1 / max;
            speed_rb *= 1 / max;
        }
        motorLF.setPower(speed_lf * powerIndex);
        motorRF.setPower(speed_rf * powerIndex);
        motorLB.setPower(speed_lb * powerIndex);
        motorRB.setPower(speed_rb * powerIndex);
    }

    /**
     * move the robot according to linear and angular speed
     * @param x vertical linear speed
     * @param y horizontal linear speed
     * @param w angular speed
     *
     * @see #movePower(double, double, double)
     */
    @Override
    public void move(double x, double y, double w) {
        double speed_lf = y + x + w;
        double speed_lb = y - x + w;
        double speed_rf = y - x - w;
        double speed_rb = y + x - w;

        double max = getBig(getBig(getBig(Math.abs(speed_lf),
                Math.abs(speed_rf)), Math.abs(speed_lb)), Math.abs(speed_rb));
        if (max > 1) {
            speed_lf *= 1 / max;
            speed_rf *= 1 / max;
            speed_lb *= 1 / max;
            speed_rb *= 1 / max;
        }
        motorLF.setVelocity(speed_lf * speedIndex);
        motorRF.setVelocity(speed_rf * speedIndex);
        motorLB.setVelocity(speed_lb * speedIndex);
        motorRB.setVelocity(speed_rb * speedIndex);
    }

    /**
     * Move the robot corresponding to the absolute coordinate system of the playing field
     * @param gamepad
     * @param angularVeolocity
     * @param odometry
     *
     * @see #move(double, double, double)
     * @see #movePower(double, double, double)
     */
    @Override
    public void moveAbsolutely(Gamepad gamepad, double angularVeolocity, ThreeWheelOdometryForYuntai odometry) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double w = angularVeolocity;
        double index = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
        double relativeReturnAngle = Math.atan2(y,x) - toRadians(odometry.angle());
        x = cos(relativeReturnAngle) * index;
        y = sin(relativeReturnAngle) * index;
        movePower(x, y, w);
    }

    /**
     * Force the robot to stop
     *
     * @see #move(double, double, double)
     * @see #movePower(double, double, double)
     */
    @Override
    public void stop() {
        movePower(0,0,0);
    }

    /**
     * Run to the target position, ends in the target position and absolute angle
     *
     * @param currentPosition  current position of the robot
     * @param target       target position that holds the coordinate and the angle in degree
     * @param linearIndex  the speed index of linear movement
     * @param angularIndex the speed index of turing movement
     */
    @Override
    public void runToPosition(Position currentPosition, Position target, double linearIndex, double angularIndex) {
        double AbsoluteAngleToPoint = Math.atan2(target.y - currentPosition.y, target.x - currentPosition.x);
        double angleToPoint = AbsoluteAngleToPoint - Math.toRadians(currentPosition.w);

        move(cos(angleToPoint) * linearIndex,
                Math.sin(angleToPoint) * linearIndex,
                Range.clip(Math.abs(getOptimizedAngle(target.w,currentPosition.w))/30,-1,1)*angularIndex);
    }
}
