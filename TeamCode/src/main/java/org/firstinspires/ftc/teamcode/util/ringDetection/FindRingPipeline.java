package org.firstinspires.ftc.teamcode.util.ringDetection;

import android.os.Build;

import org.firstinspires.ftc.teamcode.util.Position;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.PriorityQueue;

import androidx.annotation.RequiresApi;

/**
 * OpenCV pipeline to recognize rings
 *
 * @Author Candela
 */
public class FindRingPipeline extends OpenCvPipeline {
    private ArrayList<ConnectedComponent> component;
    private PriorityQueue<Ring> ring;
    public volatile boolean finishFlag;
    private static final int minWidth = 25;
    private static final int minHeight = 5;
    private static final Comparator<Ring> cmp = new Comparator<Ring>() {
        @Override
        public int compare(Ring e1, Ring e2) {
            return sgn(e1.getDistance() - e2.getDistance());
        }
    };

    public static int sgn(double n) {
        if (n > 0) return 1;
        else if (n == 0) return 0;
        else return -1;
    }

    private void bfs(Point point, Mat input) {
        if (!Color.isRingColor(input.get((int)point.y, (int)point.x))) return;

        for (ConnectedComponent cc : component) if (cc.contains(point)) return;

        component.get(component.size() - 1).add(point);

        if (point.x > 0) bfs(new Point(point.x - 1, point.y), input);
        if (point.x < input.cols() - 1) bfs(new Point(point.x + 1, point.y), input);
        if (point.y > 0) bfs(new Point(point.x, point.y - 1), input);
        if (point.y < input.rows() - 1) bfs(new Point(point.x, point.y + 1), input);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public Mat processFrame(Mat input)
    {
        input.convertTo(input, CvType.CV_8UC3);
        Mat temp = new Mat(input.size(), input.type());
        Imgproc.medianBlur(input, temp, 5);
        finishFlag = false;
        ring = new PriorityQueue<>(cmp);
        component = new ArrayList<>();
        component.add(new ConnectedComponent());

        int cols = input.cols() / minWidth;
        int rows = input.rows() / minHeight;

        for (int i = 0; i < cols; i++)
            for (int j = 0; j < rows; j++)
             {
                bfs(new Point(i * minWidth + minWidth / 2, j * minHeight + minHeight / 2), temp);
                if (!component.get(component.size() - 1).isEmpty())
                    component.add(new ConnectedComponent());
            }

        for (ConnectedComponent cc : component) {
            Ring r = new Ring(cc.left(), cc.right(), cc.up(), cc.down(), cc.downMost());
            if (r.getWidth() >= minWidth && r.getHeight() >= minHeight
                    && r.getRatio() > 0.5 && r.getRatio() < 10)
                        ring.add(r);
        }
        finishFlag = true;
        if (ring.isEmpty())
            Imgproc.putText(input,
                    "None",
                    new Point(input.cols()/2 - 20, input.rows()/2 + 60),
                    Imgproc.FONT_HERSHEY_COMPLEX,
                    0.5,
                    new Scalar(Color.GREEN.toChannel4()));
        else for (Ring r : ring)
            r.drawFrame(input);

        return input;
    }

    public PriorityQueue<Ring> getRings(){
        return ring;
    }
    public ArrayList<Position> getPositionsOfRingsInRange(int up, int down, Position current){
        ArrayList<Position> ringsInRangePos = new ArrayList<>();
        ringsInRangePos.add(current);
        for (Ring ring : getRings()){
            if(ring.getUp() > up && ring.getDown() < down) {
                Position pos = current.getExtendedPosition(ring.getPosition());
                ringsInRangePos.add(pos);
            }
        }
        return ringsInRangePos;
    }
    public ArrayList<Ring> getRingsInRange(double up, double down){
        ArrayList<Ring> ringsInRange = new ArrayList<>();
        if(getRings() != null) {
            for (Ring ring : getRings()) {
                if (ring.getUp() > up && ring.getDown() < down) ringsInRange.add(ring);
            }
        }
        return ringsInRange;
    }
}