package org.firstinspires.ftc.teamcode.util.ringDetection;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.util.Position;

public class RingTensorFlow{

    float left, right, top, bottom;
    double estimatedAngle;
    String ringCase;

    public RingTensorFlow(float left, float right, float top, float bottom, String ringCase){
        this.left = left;
        this.right = right;
        this.top = top;
        this.bottom = bottom;
        this.ringCase = ringCase;
    }

    public RingTensorFlow(float left, float right, float top, float bottom, double estimatedAngle, String ringCase){
        this.left = left;
        this.right = right;
        this.top = top;
        this.bottom = bottom;
        this.estimatedAngle = estimatedAngle;
        this.ringCase = ringCase;
    }

    public RingTensorFlow(Recognition ring){
        this.left = ring.getLeft();
        this.right = ring.getRight();
        this.top = ring.getTop();
        this.bottom = ring.getBottom();
        this.estimatedAngle = ring.estimateAngleToObject(AngleUnit.DEGREES);
        this.ringCase = ring.getLabel();

    }

        public Position getPosition(){
            float x = (left + right) / 2;
            float y = bottom;
            return new Position( (-0.105027791 * x -0.011669755 * y + 18.81747929)/(0.000413387 * x -0.013068208 * y + 1),
                    (-0.054849212 * x -0.061229618 * y -10.49356972)/(-0.000764579 * x -0.009489632
                            * y + 1));
        }

    public String toString(){
        if(ringCase == "null") {
            return ringCase;
        }else{
            return ringCase + " (" + String.format("%.2f",getPosition().x) + "," + String.format("%.2f",getPosition().y) + ")";
        }
    }
}
