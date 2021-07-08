package org.firstinspires.ftc.teamcode.util.ringDetection;

import org.firstinspires.ftc.teamcode.util.Position;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Container of ring detection that generate by FindRingPipeline
 *
 * @see     FindRingPipeline
 * @Author  Candela
 */
public class Ring extends Object{
    private Position position;
    private Point downMost;
    private String ringCase;

    private int left, right, up, down;

    public Ring(int left, int right, int up, int down, Point downMost) {
        this.left = left;
        this.right = right;
        this.up = up;
        this.down = down;
        this.downMost = downMost;
        double x = downMost.x;
        double y = downMost.y;
        this.position = new Position( (-0.105027791 * x -0.011669755 * y + 18.81747929)/(0.000413387 * x -0.013068208 * y + 1),
                (-0.054849212 * x -0.061229618 * y -10.49356972)/(-0.000764579 * x -0.009489632
                        * y + 1));
        if (1.0 * (right - left) / (down - up) > 1.7) this.ringCase = "Single";
        else this.ringCase = "Quad";
    }

    public void drawFrame(Mat mat) {
        Imgproc.rectangle(mat,
                new Point(left - 1, up - 1),
                new Point(right + 1, down + 1),
                new Scalar(Color.GREEN.toChannel4()),
                2,
                8,
                0 );
        Imgproc.rectangle(mat,
                new Point(downMost.x - 1, downMost.y - 1),
                new Point(downMost.x + 1, downMost.y + 1),
                new Scalar(Color.GREEN.toChannel4()),
                1,
                8,
                0 );
        Imgproc.putText(mat,
                getRingCase(),
                new Point(left - 3, down + 15),
                Imgproc.FONT_HERSHEY_COMPLEX,
                0.5,
                new Scalar(Color.GREEN.toChannel4()));
    }

    public double getDistance() {
        return Math.sqrt(Math.pow(position.x, 2) + Math.pow(position.y, 2));
    }
    public Position getPosition() {
        return position;
    }
    public int getWidth() {
        return right - left;
    }
    public int getHeight() {
        return down - up;
    }
    public double getRatio() {
        return 1.0 * getWidth() / getHeight();
    }
    public String getRingCase() {
        return ringCase;
    }
    public int getLeft(){
        return  left;
    }
    public int getRight(){
        return right;
    }
    public int getUp(){
        return up;
    }
    public int getDown() {
        return down;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Ring) {
            Ring r = (Ring)obj;
            if (r.left == this.left && r.right == this.right && r.up == this.up && r.down == this.down)
                return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "Ring " + ringCase + " at (" + position.x + ", " + position.y + ")";
    }
}