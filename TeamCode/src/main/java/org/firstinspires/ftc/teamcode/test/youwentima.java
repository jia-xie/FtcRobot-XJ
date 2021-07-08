package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.util.MathFunction.getBig;

@Disabled
@TeleOp
public class youwentima extends OpMode {

    DcMotorEx motorLF, motorLB, motorRF, motorRB;

    @Override
    public void init() {
        motorLF = hardwareMap.get(DcMotorEx.class, "LF");
        motorRF = hardwareMap.get(DcMotorEx.class, "RF");
        motorLB = hardwareMap.get(DcMotorEx.class, "LB");
        motorRB = hardwareMap.get(DcMotorEx.class, "RB");

        motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);

        motorLF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
//        double x = gamepad1.left_stick_x;
//        double y = -gamepad1.left_stick_y;
//        double w = gamepad1.right_stick_x;
//
//        double speed_lf = y + x + w;
//        double speed_lb = y - x + w;
//        double speed_rf = y - x - w;
//        double speed_rb = y + x - w;
//
//        double max = getBig(getBig(getBig(Math.abs(speed_lf),
//                Math.abs(speed_rf)), Math.abs(speed_lb)), Math.abs(speed_rb));
//        if (max > 1) {
//            speed_lf *= 1 / max;
//            speed_rf *= 1 / max;
//            speed_lb *= 1 / max;
//            speed_rb *= 1 / max;
//        }
//        motorLF.setPower(speed_lf * .8);
//        motorRF.setPower(speed_rf * .8);
//        motorLB.setPower(speed_lb * .8);
//        motorRB.setPower(speed_rb * .8);
        motorLF.setPower(1);
    }
}
