package org.firstinspires.ftc.teamcode.util;

import org.jetbrains.annotations.NotNull;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static org.firstinspires.ftc.teamcode.util.MathFunction.getAngle;

/**
 * Container of position, which include the coordinate across the playing filed and the heading angle.
 *
 * @Author Jason Xie
 */
public class Position {
    public double x;
    public double y;
    public double w;   //in DEGREE
    public double w_init;

    public Position(){
        this.x = 0;
        this.y = 0;
        this.w = 90;
    }

    public Position (double x, double y){
        this.x = x;
        this.y = y;
    }

    public Position (double x, double y, double w, double w_init){
        this.x = x;
        this.y = y;
        this.w = w;
        this.w_init = w_init;
    }

    public Position (double x, double y, double w){
        this.x = x;
        this.y = y;
        this.w = w;

    }

    public double getDistance(Position other){
        double result =
                Math.sqrt(Math.pow((other.y-this.y),2)+Math.pow((other.x-this.x),2));
        return result;
    }

    public double getAbsoluteAngle(){
        return toDegrees(atan2(y,x));
    }

    public Position getRelativePosition(Position target){
        return new Position(target.x - x, target.y - y);
    }

    @Deprecated
    public Position getCoordinatePosition(){
        return new Position(x,y,w+90);
    }

    /**
     * get the absolute angle towards the target position, in degree
     * @param   target    the target position that look ahead to
     * @return  the angle towards the target positions
     */
    public double getRelativeAngle(@NotNull Position target){
        return getAngle(target, this);
    }

    /**
     * get the extended the position, by add the vector position
     * @param   relativePosition    the adding position
     * @return  the extended position
     *
     * @see #getExtendedPosition(double, double)
     */
    public Position getExtendedPosition(Position relativePosition){
        double distance = getDistance(relativePosition);
        double angle = Math.atan2(relativePosition.y,relativePosition.x);

        return new Position(x + distance * cos(angle),
                y + distance * sin(angle),
                0);
    }

    /**
     * get the extended position, by add the vector position (x , y).
     * @param x the x coordinate of the adding position
     * @param y the y coordinate of the adding position
     * @return  the extended position
     *
     * @see #getExtendedPosition(Position)
     */
    public Position getExtendedPosition(double x, double y){
        return getExtendedPosition(new Position(x,y));
    }

    /**
     * set this.position as the same as the parameter positoin
     * @param other
     */
    public void setPosition(@NotNull Position other){
        this.x = other.x;
        this.y = other.y;
        this.w = other.w;
    }

    /**
     * set this.position to (x, y, w)
     * @param x
     * @param y
     * @param w
     */
    public void setPosition(double x, double y, double w){
        this.x = x;
        this.y = y;
        this.w = w;
    }

    /**
     * set the x and y coordinate as the parameter
     * @param x
     * @param y
     */
    public void setPosition(double x, double y){
        this.x = x;
        this.y = y;
    }
    public void setX(double x){
        this.x = x;
    }
    public void setY(double y){
        this.y = y;
    }
    public void setRelativeX(double relativeX){
        x += relativeX;
    }

    public Position setRelativeY(double relativeY){
        y += relativeY;
        return this;
    }
    public void setAngle(double w){
        this.w = w;
    }

    /**
     * output the x,y,w in the format of (x,y,w), to easily debug
     * @return
     */
    public String toString(){
        return "(" + String.format("%.2f",x) + "," + String.format("%.2f",y) + "," + String.format("%.2f",w) + ")";
    }
}
