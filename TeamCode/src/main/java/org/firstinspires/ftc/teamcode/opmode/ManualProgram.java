package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.GameCircumstance;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;
import org.firstinspires.ftc.teamcode.util.robotAssembly.DriveTrainTeleOpModule;

@TeleOp(group = "OpMode", name = "OpMode: Candela TeleOp")
@Config
public class ManualProgram extends OpMode {
    public static double pushForwardIndex = .5;
    public static double pushBackwardIndex = .1;
    public static double upIndex = .6;
    public static double downIndex = .31;
    public static double pickCloseIndex = 1;
    public static double pickHalfCloseIndex = .6;
    public static double pickOpenIndex = .3;
    public static double deliveryDownIndex = .3;
    public static double deliveryUpIndex = 1;
    public static double speedEject = 0;
    public static double highGoalEjectPower = 1;
    //Motor Index
    public static double highGoalEjectSpeed = 2350;
    public static double powerEjectSpeedI = 2000;
    public static double powerEjectPowerII = .7;
    public static double takeInPowerIndex = -1;

    DcMotorEx motorEjectI, motorEjectII_and_RightVertical, motorTakeIn_and_Horizontal, motorLeftVertical;
    Servo servoPush, servoUpDown, servoDelivery, servoPick;
    DistanceSensor distanceSensor;
    ThreeWheelOdometryForYuntai odometry;
    DriveTrainTeleOpModule driveTrainTeleOpModule;

    @Override
    public void init() {
        initServoMotor();

        driveTrainTeleOpModule = new DriveTrainTeleOpModule(
                gamepad1, gamepad2, hardwareMap,
                2000, GameCircumstance.AllianceColor.RED, odometry);
        new Thread(driveTrainTeleOpModule).start();


        odometry = new ThreeWheelOdometryForYuntai(motorLeftVertical,
                motorEjectII_and_RightVertical,
                motorTakeIn_and_Horizontal,183,
                110.125, 9,0,
                ThreeWheelOdometryForYuntai.UpdateType.LINEAR);
        odometry.resetPulseRecord();
        new Thread(odometry).start();

        distanceSensor = hardwareMap.get(DistanceSensor.class, "i1");

    }

    @Override
    public void loop() {

        takeIn();
        launchMotor();
        servoMove();

        telemetry.addData("Flag", driveTrainTeleOpModule.getAngleLockSwitch());
        telemetry.addLine(odometry.getCurrentPosition().toString());
        telemetry.update();
    }

    @Override
    public void stop(){
        driveTrainTeleOpModule.stopThread();
    }
    private void takeIn(){
        if(gamepad1.y){
            servoUpDown.setPosition(upIndex);
        }
        if (gamepad1.left_bumper){
            motorTakeIn_and_Horizontal.setPower(-takeInPowerIndex);
        } else if(gamepad1.right_bumper) {
            if (distanceSensor.getDistance(DistanceUnit.MM) < 55) {
                servoUpDown.setPosition(upIndex);
                motorTakeIn_and_Horizontal.setPower(-takeInPowerIndex);
            }else{
                motorTakeIn_and_Horizontal.setPower(takeInPowerIndex);
                servoUpDown.setPosition(downIndex);
            }
        }else{
            motorTakeIn_and_Horizontal.setPower(0);
        }
    }
    private void launchMotor(){
        if(gamepad1.right_stick_button){
            motorEjectI.setVelocity(highGoalEjectSpeed);
            motorEjectII_and_RightVertical.setPower(highGoalEjectPower);

        }else if(gamepad1.left_stick_button){
            motorEjectI.setPower(0);
            motorEjectII_and_RightVertical.setPower(0);
        }else if(gamepad1.left_trigger>0.5){
            motorEjectI.setVelocity(powerEjectSpeedI);
            motorEjectII_and_RightVertical.setPower(powerEjectPowerII);
        }

    }
    private void servoMove(){
        //Push
        if(gamepad1.x){
            servoPush.setPosition(pushForwardIndex);
        }else{
            servoPush.setPosition(pushBackwardIndex);
        }

        if(gamepad1.dpad_down){
            servoUpDown.setPosition(downIndex);
        }else if (gamepad1.dpad_up){
            servoUpDown.setPosition(upIndex);
        }

        //wobble goal
        if(gamepad2.dpad_down){
            servoDelivery.setPosition(deliveryDownIndex);
        }
        if(gamepad2.dpad_up){
            servoDelivery.setPosition(deliveryUpIndex);
        }
        if(gamepad2.a){
            servoPick.setPosition(pickCloseIndex);
        }
        if(gamepad2.b){
            servoPick.setPosition(pickOpenIndex);
        }
        if (gamepad2.y){
            servoDelivery.setPosition(.4);
        }
        if (gamepad2.x){
            servoPick.setPosition(pickHalfCloseIndex);
        }
    }
    private void initServoMotor(){
        motorEjectI = hardwareMap.get(DcMotorEx.class,"m20");
        motorEjectII_and_RightVertical = hardwareMap.get(DcMotorEx.class, "m21");
        motorLeftVertical = hardwareMap.get(DcMotorEx.class, "m23");
        motorTakeIn_and_Horizontal = hardwareMap.get(DcMotorEx.class,"m22");

        servoPush = hardwareMap.get(Servo.class,"sP");
        servoUpDown = hardwareMap.get(Servo.class,"sU");
        servoDelivery = hardwareMap.get(Servo.class, "sD");
        servoPick = hardwareMap.get(Servo.class, "sPick");
    }
}
