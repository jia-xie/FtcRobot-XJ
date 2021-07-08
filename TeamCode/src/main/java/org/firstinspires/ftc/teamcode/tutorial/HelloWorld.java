package org.firstinspires.ftc.teamcode.tutorial;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.util.MathFunction.getBig;

@TeleOp
public class HelloWorld extends OpMode {

    DcMotor motorLF, motorRF, motorRB, motorLB;



    @Override
    public void init() {
        motorLF = hardwareMap.get(DcMotor.class, "m10");
        motorRF = hardwareMap.get(DcMotor.class, "m11");
        motorRB = hardwareMap.get(DcMotor.class, "m12");
        motorLB = hardwareMap.get(DcMotor.class, "m13");

        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
//        if(){
//
//        }else if(){
//
//        }else{
//
//        }
        movePower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);


    }

    public void movePower(double x, double y, double w) {
        double speed_lf = y + x + w;
        double speed_lb = y - x + w;
        double speed_rf = y - x - w;
        double speed_rb = y + x - w;

        double max = getBig(getBig(getBig(Math.abs(speed_lf),
                Math.abs(speed_rf)), Math.abs(speed_lb)), Math.abs(speed_rb));
        if (max > 1) {
            speed_lf *= 1 / max;
            speed_rf *= 1 / max;
            speed_lb *= 1 / max;
            speed_rb *= 1 / max;
        }
        motorLF.setPower(speed_lf * 0.8);
        motorRF.setPower(speed_rf * 0.8);
        motorLB.setPower(speed_lb * 0.8);
        motorRB.setPower(speed_rb * 0.8);
    }
}
