package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.control.AngularPID;

@TeleOp
@Config
public class LaunchTestOpMode extends OpMode {

    DcMotor motorLaunchI, motorLaunchII;
    Servo servoDirection, servoPush;
    double launchAngle = 90;
    public static double forwardIndex = 0.5;
    public static double backIndex = 0.9;
    public static double launchPower = 1;
    public static double upIndex = 0;
    public static double almostUpIndex = 0.05;
    public static double downIndex = 1;
    Servo servoUpDown;

    @Override
    public void init() {

        motorLaunchI = hardwareMap.get(DcMotor.class, "m10");
        motorLaunchII = hardwareMap.get(DcMotor.class, "m11");

        servoPush = hardwareMap.get(Servo.class, "s10");
        servoDirection = hardwareMap.get(Servo.class, "s11");
        servoUpDown = hardwareMap.get(Servo.class, "s12");

        motorLaunchII.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            setLaunchPower(launchPower);
        }else if(gamepad1.b){
            setLaunchPower(0);
        }

        if(gamepad1.dpad_left){
            launchAngle -= 0.03;
        }else if(gamepad1.dpad_right){
            launchAngle  += 0.03;
        }

        servoDirection.setPosition(launchAngle/180.0);

        if(gamepad1.x){
            servoPush.setPosition(forwardIndex);
        }else{
            servoPush.setPosition(backIndex);
        }

        if(gamepad1.dpad_up) {
            servoUpDown.setPosition(upIndex);
        }
        if (gamepad1.dpad_down){
            servoUpDown.setPosition(downIndex);
        }
        if (gamepad1.y){
            servoUpDown.setPosition(almostUpIndex);
        }
    }

    private void setLaunchPower(double powerIndex){
        motorLaunchI.setPower(powerIndex);
        motorLaunchII.setPower(powerIndex);
    }
}
