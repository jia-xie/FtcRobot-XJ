package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class TakeInTestOpMode extends OpMode {

    DcMotor motorTimingBelt, motorRubberPipe;
    public static double timingBeltPower = -1;
    public static double rubberPipePower = -1;
    @Override
    public void init() {
        motorRubberPipe = hardwareMap.get(DcMotor.class, "m10");
        motorTimingBelt = hardwareMap.get(DcMotor.class, "m11");
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper){
            motorTimingBelt.setPower(timingBeltPower);
            motorRubberPipe.setPower(rubberPipePower);
        }else if(gamepad1.left_bumper){
            motorTimingBelt.setPower(-timingBeltPower);
            motorRubberPipe.setPower(-rubberPipePower);
        }else{
            motorTimingBelt.setPower(0);
            motorRubberPipe.setPower(0);
        }


    }
}
