package org.firstinspires.ftc.teamcode.util.control;

import org.firstinspires.ftc.teamcode.util.Position;

public class PositionalPID {

    double kp;
    double ki;
    double kd;
    public Position target;
    public double error;
    double error_last;
    double dead_zone;
    double error_iteg;
    double error_iteg_range;
    double positionalPIDOut;
    double out_max;
    double finish_range;
    public boolean finish_flag=false;

    /**
     * Construct a positional pid that aim to calculate distance of *Position* instance
     * @param kp
     * @param ki
     * @param kd
     * @param dead_zone
     * @param out_max
     * @param finish_range
     * @param error_iteg_range
     */
    public PositionalPID(double kp, double ki, double kd,
                         double dead_zone, double out_max, double finish_range, double error_iteg_range) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.dead_zone = dead_zone;
        this.out_max = out_max;
        this.finish_range=finish_range;
        this.error_iteg_range = error_iteg_range;
        error_last = 0;
        error_iteg = 0;
    }

    /**
     * set the finish range of the pid controller
     * @param finish_range  the range within which the pid will output 0;
     */
    public void setFinishRange(double finish_range){
        this.finish_range = finish_range;
    }

    /**
     * update the target position that the pid controller will calculate the distance to it as the error
     * @param target
     */
    public void positionalPIDUpdate(Position target) {

        this.target = target;
        error_last = 0;
        error_iteg = 0;
        this.finish_flag = false;
    }
    public double getPositionalPIDOut(Position currentPosition) {

        //load error
        error = target.getDistance(currentPosition);

        //check error
        if (Math.abs(error) < dead_zone) error = 0;

        //set finish flag
        if(Math.abs(error) < finish_range) finish_flag = true; else finish_flag = false;

        if (Math.abs(error) < error_iteg_range) error_iteg += error;

        //////////////////////////////////////////////////////////////////////////////////
        positionalPIDOut = kp * error + error_iteg * ki + (error - error_last) * kd; /////  FORMULA
        //////////////////////////////////////////////////////////////////////////////////

        if (positionalPIDOut > out_max) positionalPIDOut = out_max;
        else if (positionalPIDOut < -out_max) positionalPIDOut = -out_max;

        error_last = error;
        if(finish_flag)return 0;
        return  positionalPIDOut;
    }


}
