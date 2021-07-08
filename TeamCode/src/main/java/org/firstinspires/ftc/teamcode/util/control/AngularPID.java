package org.firstinspires.ftc.teamcode.util.control;

import static org.firstinspires.ftc.teamcode.util.MathFunction.getOptimizedAngle;

public class AngularPID {

    double kp;
    double ki;
    double kd;
    double target;
    double error;
    double error_last;
    double dead_zone;
    double error_iteg;
    double error_iteg_range;
    double AngularPIDOut;
    double out_max;
    double out_min;
    double finish_range;
    public boolean finish_flag=false;

    public AngularPID(double kp, double ki, double kd,
                      double dead_zone, double out_max, double out_min, double finish_range, double error_iteg_range) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.dead_zone = dead_zone;
        this.out_max = out_max;
        this.out_min = out_min;
        this.finish_range=finish_range;
        this.error_iteg_range = error_iteg_range;
        error_last = 0;
        error_iteg = 0;
    }


    public void angularPIDUpdate(double target) {

        this.target = target;
        error_last = 0;
        error_iteg = 0;
        this.finish_flag=false;
    }
    public void angularPIDUpdateOnlyForTheSigma(double target) {

        this.target = target;
        error_last = 0;
        error_iteg = 0;
    }
    public double getAngularPIDOut(double currentAngle) {

        //load error
        error =  -getOptimizedAngle(target,currentAngle);

        if(error<-180) error+=360;
        if(error>180) error-=360;

        //check error
        if (Math.abs(error) < dead_zone) error = 0;

        //set finish flag
        if(Math.abs(error) < finish_range) finish_flag = true; else finish_flag = false;

        if (Math.abs(error) < error_iteg_range) error_iteg += error;

        //////////////////////////////////////////////////////////////////////////////////
        AngularPIDOut = kp * error + error_iteg * ki + (error - error_last) * kd; //////// FORMULA
        //////////////////////////////////////////////////////////////////////////////////

        if (AngularPIDOut > out_max) AngularPIDOut = out_max;
        else if (AngularPIDOut < -out_max) AngularPIDOut = -out_max;


        if (Math.abs(AngularPIDOut) < out_min) AngularPIDOut = (Math.abs(AngularPIDOut) / AngularPIDOut) * out_min;

        error_last = error;
        return AngularPIDOut;
    }


}
