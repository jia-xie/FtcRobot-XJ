package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(group = "test", name = "test: fashe")
@Config
public class FasheTest extends LinearOpMode {
    public static double positionIndex = 0.5;
    public static double powerIndex = 1;
    public static double pushIndex = 0;
    public static double backIndex = 0.5;
    DcMotorEx mEjectL;
    DcMotorEx mEjectR;
    Servo sEjectR;
    Servo sPush;

    @Override
    public void runOpMode() throws InterruptedException {
        mEjectL = hardwareMap.get(DcMotorEx.class,"m10");
        mEjectR = hardwareMap.get(DcMotorEx.class,"m11");

        sEjectR = hardwareMap.get(Servo.class,"s10");
        sPush = hardwareMap.get(Servo.class, "s11");

        waitForStart();
        eject(powerIndex);
        sEjectR.setPosition(positionIndex);


        while(opModeIsActive() && !isStopRequested()){
            sPush.setPosition(pushIndex);
            sleep(300);
            sPush.setPosition(backIndex);
            sleep(1000);
        };
    }

    public void eject(double power){
        mEjectR.setPower(-power);
        mEjectL.setPower(power);
    }
}
