package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedList;

public class MyQueue extends LinkedList<Double> {
    int depth;
    public MyQueue(int depth) {
        this.depth = depth;
    }

    @Override
    public boolean offer(Double x) {
        if (size() == depth) {
            add(x);
            remove();
        }
        else add(x);
        return true;
    }

    public double average() {
        double ans = 0;
        int num = 0;
        for (Double x : this) {
            ans += x;
            num ++;
        }
        if (num > 0) return ans / num;
        return 0;
    }
}
