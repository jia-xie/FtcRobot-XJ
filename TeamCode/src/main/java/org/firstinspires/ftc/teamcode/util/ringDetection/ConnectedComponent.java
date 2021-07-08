package org.firstinspires.ftc.teamcode.util.ringDetection;

import org.opencv.core.Point;

import java.util.HashSet;

public class ConnectedComponent extends Object{
    private HashSet<Point> point;
    private int left, right, up, down;

    public ConnectedComponent() {
        point = new HashSet<>();
        left = right = up = down = -5;
    }

    public int size() {
        return point.size();
    }

    public boolean isEmpty() {
        return point.isEmpty();
    }

    public void add(Point p) {
        point.add(p);
        left = (left < 0) ? (int)p.x : Math.min(left, (int)p.x);
        right = (right < 0) ? (int)p.x : Math.max(right, (int)p.x);
        up = (up < 0) ? (int)p.y : Math.min(up, (int)p.y);
        down = (down < 0) ? (int)p.y : Math.max(down, (int)p.y);
    }

    public int left() {
        return left;
    }
    public int right() {
        return right;
    }
    public int up() {
        return up;
    }
    public int down() {
        return down;
    }
    public Point downMost() {
        double x = 0;
        int t = 0;
        for (Point p : point)
            if (p.y >= down - 3) {
                x += p.x;
                t++;
            }
        x /= t;
        return new Point(x, down);
    }

    public boolean contains(Point p) {
        return point.contains(p);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof ConnectedComponent) return point.equals(((ConnectedComponent)obj).point);
        return false;
    }
}