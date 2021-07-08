package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Position;

import java.util.ArrayList;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.floor;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

/**
 *All the math functions
 *
 * @Author Candela
 */
public class MathFunction {


    public static double getBig(double x, double y) {
        return x > y ? x : y;
    }
    public static double getSmall(double x, double y){
        return x < y ? x : y;
    }

    /**
     * return the angle of the vector (current, target), in degree
     * @param target
     * @param current
     * @return
     */
    public static double getAngle(Position target, Position current){
        return toDegrees(Math.atan2(target.y - current.y, target.x - current.x));
    }
    public static double getTan(Position target, Position current){

        double x_Difference = target.x - current.x;
        double y_Difference = target.y - current.y;
        if (y_Difference < 0.003){
            y_Difference = 0.003;
        }
        if (x_Difference < 0.003){
            x_Difference = 0.003;
        }
        double tan = y_Difference/x_Difference;
        return tan;
    }


    /**
     * get the intersection points (hold in an ArrayList of Position instances) of a line and a circle
     * @param linePoint1    the first end point of a line
     * @param linePoint2    the second end point of a line
     * @param circleCenter  the center point of the circle
     * @param radius        the radius of the circle
     * @return              An ArrayList of intersection points, which is in between the two end points
     */
    public static ArrayList<Position> getIntersectionPosition
            (Position linePoint1, Position linePoint2,
             Position circleCenter, double radius){

        ArrayList<Position> positions = new ArrayList<>();

        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003){
            linePoint1.x = linePoint2.x + 0.003;
        }
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003){
            linePoint1.y = linePoint2.y + 0.003;
        }

        double m1 = (linePoint1.y - linePoint2.y) / (linePoint1.x - linePoint2.x);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double A = 1.0 + Math.pow(m1,2);
        double B = (2.0 * m1 * y1) - (2.0 * Math.pow(m1,2) * x1);
        double C = ((Math.pow(m1,2) * Math.pow(x1,2))) - (2.0*y1*m1*x1)+ pow(y1,2)- pow(radius,2);

        try{
            double xRoot1 = (-B + sqrt(pow(B,2) - (4.0 * A * C )))/(2.0 * A);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double xMax = getBig(linePoint1.x,linePoint2.x);
            double xMin = getSmall(linePoint1.x,linePoint2.x);
            double yMax = getBig(linePoint1.y,linePoint2.y);
            double yMin = getSmall(linePoint1.y,linePoint2.y);
            if (xRoot1 > xMin && xRoot1 < xMax && yRoot1 > yMin && yRoot1 < yMax){
                positions.add(new Position(xRoot1,yRoot1));
            }

            double xRoot2 = (-B - sqrt(pow(B,2) - (4.0 * A * C )))/(2.0 * A);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if (xRoot2 > xMin && xRoot2 < xMax && yRoot2 > yMin && yRoot2 < yMax){
                positions.add(new Position(xRoot2,yRoot2));
            }
        }catch (Exception e){

        }
        return positions;
    }

    /**
     * Get the position that is closer to the current angle of the robot. The algorithm is to
     * prevent the robot to move to the false position that #getIntersectionPosition() calculated,
     * since getIntersectionPosition can only calculate the mathematical intersection position
     * without consideration of robot heading angle.
     * @param positions         An ArrayList that holds the two positions that is about to be compared
     * @param currentPosition   The current position of the robot
     * @param reverseFlag       A flag show if the robot is move forward or backward
     * @return                  The position that is closer to the current angle of the robot
     *
     */
    public static Position getClosestAnglePosition
            (ArrayList<Position> positions, Position currentPosition, boolean reverseFlag, Telemetry telemetry){
        Position pos1 = positions.get(0);
        Position pos2 = positions.get(1);
        double R1 = Math.atan2(pos1.y - currentPosition.y, pos1.x - currentPosition.x);
        R1 = getOptimizedAngle(toDegrees(R1),currentPosition.w + 90);
        double R2 = Math.atan2(pos2.y - currentPosition.y, pos2.x - currentPosition.x);
        R2 = getOptimizedAngle(toDegrees(R2),currentPosition.w + 90);

        if(!reverseFlag) {
            if (abs(R1) < abs(R2)) {
                return pos1;
            } else {
                return pos2;
            }
        }else{
            if (abs(R1) > abs(R2)) {
                return pos1;
            } else {
                return pos2;
            }
        }

    }


    /**
     * Get the next position that the robot is supposed to move towards. When there is two intersection
     * position between the robot circle the target line, it will follow the position that is closer
     * to the current angle of the robot.
     *
     * Notice:This method is used when applying to PurePursuit algorithm.
     *
     * @param pathPositions     An ArrayList of positions on the target path
     * @param currentPosition   The Current position of the robot
     * @param desiredRadius     The radius of the PurePursuit circle
     * @param reverseFlag       A flag show if the robot is move forward or backward
     * @return                  The next position that the robot is supposed to move towards
     */
    public static Position getFollowPosition(ArrayList<Position> pathPositions,
                                             Position currentPosition, double desiredRadius,
                                             boolean reverseFlag,
                                             Telemetry telemetry){
        Position followPosition = new Position();
        for (int i = 0 ; i < pathPositions.size() - 1 ; i ++){
            Position pos1 = pathPositions.get(i);
            Position pos2 = pathPositions.get(i + 1);
            ArrayList<Position> followPositions = getIntersectionPosition(pos1,pos2,currentPosition,desiredRadius);
            if (followPositions.size() == 1){
                followPosition.setPosition(followPositions.get(0));
            }
            if(followPositions.size() == 2){
                followPosition.setPosition(getClosestAnglePosition(followPositions,currentPosition, reverseFlag,telemetry));
//                telemetry.addData("Pos1",followPositions.get(0).toString());
//                telemetry.addData("Pos2x",followPositions.get(1).toString());
            }
        }
//        telemetry.addData("follow",followPosition.toString());
        return followPosition;
    }

    public static double getOptimizedAngle(Position target, Position currentPosition){

        double desiredAngle = getAngle(target,currentPosition);
        if (Math.abs(desiredAngle) > 180){
            if (desiredAngle>0){
                desiredAngle -= 360;
            }else{
                desiredAngle += 360;
            }
        }

        return desiredAngle;

    }

    public static double getOptimizedAngle(double targetAngle, double currentAngle){

        double desiredAngle = targetAngle - currentAngle;
        while(Math.abs(desiredAngle) > 180){
            if (desiredAngle > 0){
                desiredAngle -=360;
            }else{
                desiredAngle +=360;
            }
        }

        return desiredAngle;

    }

    public static double getOptimizedAngle(double desiredAngle){
        while(Math.abs(desiredAngle) > 180){
            if (desiredAngle > 0){
                desiredAngle -=360;
            }else{
                desiredAngle +=360;
            }
        }

        return desiredAngle;
    }
}
