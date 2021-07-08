package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.control.AngularPID;
import org.firstinspires.ftc.teamcode.util.robotAssembly.DriveTrainMotionManagement;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.control.PositionalPID;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;


import static java.lang.Math.abs;

@Config
@TeleOp(group = "OpMode",name = "OpMode: Candela OpMode")

public class ObsoleteManualProgram extends OpMode {

    FtcDashboard dashboard;
    //Parameters
    int speedMax = 2000;
    double x = 0;
    double y = 0;
    double w = 0;

    ThreeWheelOdometryForYuntai odometry;
    ElapsedTime runtime = new ElapsedTime();
    //Servo Index
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
    public static double highGoalEjectSpeed = 2450;
    public static double powerEjectSpeedI = 2100;
    public static double powerEjectPowerII = .8;
    public static double powerTakeIn = -1;

    AngularPID angularRoughPID = new AngularPID(0.08, 0, 0.00, 0, 1, 0.03, 0.5, 0);
    PositionalPID positionalPID = new PositionalPID(0.09,0,0,.3,1,0.8,0);
    Position returnPosition = new Position(100.5,90,10);


    public static double speedFollow = 0;

    //Motors and Servos
    DriveTrainMotionManagement driveTrain = new DriveTrainMotionManagement();

    DcMotorEx motorEjectI = null;
    DcMotorEx motorEjectII_and_RightVertical = null;
    DcMotorEx motorLeftVertical = null;
    DcMotorEx motorTakeIn_and_Horizontal = null;

    Servo servoPush = null;
    Servo servoUpDown = null;
    Servo servoDelivery = null;
    Servo servoPick = null;

    DistanceSensor distanceSensor;
    boolean isEjectingSingle = false;
    boolean isReturningToEjectPosition = false;


    //PID Controllers
    AngularPID angularPID = new AngularPID(0.04, 0, 0.003, 0, 1, 0, 1, 0);
    AngularPID angularAccuratePID = new AngularPID(0.04, 0, 0.003, 0, 1, 0.05, 0.1, 0);



    double targetAngle = 10;
    @Override
    public void init() {
        dashboard =  FtcDashboard.getInstance();

        driveTrain.initDriveTrain(hardwareMap,speedMax);

        motorEjectI = hardwareMap.get(DcMotorEx.class,"m20");
        motorEjectII_and_RightVertical = hardwareMap.get(DcMotorEx.class, "m21");
        motorLeftVertical = hardwareMap.get(DcMotorEx.class, "m23");
        motorTakeIn_and_Horizontal = hardwareMap.get(DcMotorEx.class,"m22");

        odometry = new ThreeWheelOdometryForYuntai(motorLeftVertical,
                motorEjectII_and_RightVertical,
                motorTakeIn_and_Horizontal,183,
                110.125, 9,0,
                ThreeWheelOdometryForYuntai.UpdateType.LINEAR);

        odometry.resetPulseRecord();

        servoPush = hardwareMap.get(Servo.class,"sP");
        servoUpDown = hardwareMap.get(Servo.class,"sU");
        servoDelivery = hardwareMap.get(Servo.class, "sD");
        servoPick = hardwareMap.get(Servo.class, "sPick");

        distanceSensor = hardwareMap.get(DistanceSensor.class,"i1");
        angularPID.angularPIDUpdate(0);
        angularPID.finish_flag = true;
        positionalPID.positionalPIDUpdate(odometry.getCurrentPosition());
        angularRoughPID.angularPIDUpdate(odometry.angle());

    }

    @Override
    public void loop() {
        controllerUpdate();
        odometry.updatePositionByStraightPath();
        //Drive Train Control
//        driveTrain();
        if(angularPID.finish_flag && positionalPID.finish_flag) driveTrain.move(x, y, w);
        autoPositioning();


        //Motor Control
        motorMove();

        //Servo Control
        servoMove();

        takeInSystem();


        autoReturn();



        speedFollow = motorEjectI.getVelocity();

//        telemetry.addData("Speed Follow", speedFollow);
        speedEject = motorEjectI.getVelocity();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Distance", distanceSensor.getDistance(DistanceUnit.MM));
        dashboard.sendTelemetryPacket(packet);
//        telemetry.addData("X",odometry.getRobotX());
//        telemetry.addData("Y",odometry.getRobotY());
//        telemetry.addData("speedI",speedEject);
//        telemetry.addData("Power",motorEjectI.getPower());
//        telemetry.addData("Right", odometry.getYRightPulse());
//        telemetry.addData("Left", odometry.getYLeftPulse());
//        telemetry.addData("Horizontal", odometry.getXPulse());
//        telemetry.addData("Angle",odometry.angle());



        telemetry.update();
    }

    private void controllerUpdate(){

        if(!gamepad1.atRest() || gamepad1Dpad()){
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            w = gamepad1.right_stick_x;
            positionalPID.finish_flag = true;
            angularPID.finish_flag = true;
            isReturningToEjectPosition = false;

        }else{
            x = gamepad2.left_stick_x;
            y = -gamepad2.left_stick_y;
            w = gamepad2.right_stick_x;
        }

        if(gamepad1.a){
            positionalPID.finish_flag = false;
            isReturningToEjectPosition = true;
        }

        if(gamepad1.b){
            positionalPID.positionalPIDUpdate(odometry.getCurrentPosition());
            angularRoughPID.angularPIDUpdate(odometry.angle());
            returnPosition.setPosition(odometry.getCurrentPosition());
        }


        if(gamepad1Dpad()){
            if(gamepad1.dpad_left){
                w = -0.2;
            }else if(gamepad1.dpad_right){
                w = 0.2;
            }
        }
        if(gamepad1.right_trigger > 0.5){
            x *= 0.4;
            y *= 0.4;
            w *= 0.4;
        }

    }
//    private void driveTrain(){
//        if(angleLockSwitch){
//            if(runtime.milliseconds()>updateTimeInterval){
//                angularPID.angularPIDUpdate(getLaunchAngle(odometry));
//                runtime.reset();
//            }
//            double angularVelocity = angularPID.getAngularPIDOut(odometry.angle());
//            if(!gamepad1.atRest() || angleLockSwitch){
//                driveTrain.moveAbsolutely(gamepad1, angularVelocity, odometry);
//            }else{
//                driveTrain.stop();
//            }
//        }else{
//            if(!gamepad1.atRest()){
//                driveTrain.movePower(x, y, w);
//            }else{
//                driveTrain.stop();
//            }
//        }
//    }

    private void takeInSystem(){
        //UpDown
        if(gamepad1.y){
            servoUpDown.setPosition(upIndex);
        }

        //gamepad 1 down
        if (gamepad1.left_bumper){
            motorTakeIn_and_Horizontal.setPower(-powerTakeIn);
        }
        else if(gamepad1.right_bumper) {
            if (distanceSensor.getDistance(DistanceUnit.MM) < 72) {
                servoUpDown.setPosition(upIndex);
                motorTakeIn_and_Horizontal.setPower(-powerTakeIn);
            }else{
                motorTakeIn_and_Horizontal.setPower(powerTakeIn);
                servoUpDown.setPosition(downIndex);
            }
        }else{
            motorTakeIn_and_Horizontal.setPower(0);
        }
    }
    private void motorMove(){
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

        if(gamepad1.dpad_up){
            servoUpDown.setPosition(upIndex);
        }else if(gamepad1.dpad_down){
            servoUpDown.setPosition(downIndex);
        }

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

    private void autoPositioning(){
        if(gamepad1.b){
            targetAngle = odometry.angle();
        }
        if(Math.abs(gamepad1.right_stick_x) > 0.01){
            angularPID.finish_flag = true;
        }
        if(gamepad1.a){
            angularPID.angularPIDUpdate(targetAngle);
        }
        if(!angularPID.finish_flag){
            double w = angularPID.getAngularPIDOut(odometry.angle());
            driveTrain.move(gamepad1.left_stick_x , -gamepad1.left_stick_y, w);
        }
    }

    private void ejectSingle(){

        if(runtime.milliseconds() < 100) {
            servoPush.setPosition(pushForwardIndex);
        }else if(runtime.milliseconds() < 250){
            servoPush.setPosition(pushBackwardIndex);
            isEjectingSingle = false;
        }

    }

//    private void tripleEject(int timeInterval){
//        if(isEjectingTriple || !gamepad1.b) {
//
//            if(runtime.milliseconds() < 150){
//                servoPush.setPosition(pushForwardIndex);
//            }else if(runtime.milliseconds() < 300){
//                servoPush.setPosition(pushBackwardIndex);
//            }
//
//            else if(runtime.milliseconds()<450){
//                servoPush.setPosition(pushForwardIndex);
//            }else if(runtime.milliseconds()<600){
//                servoPush.setPosition(pushBackwardIndex);
//            }
//
//            else if(runtime.milliseconds() < 750) {
//                servoPush.setPosition(pushForwardIndex);
//            }
//            else if(runtime.milliseconds() < 900) {
//                servoPush.setPosition(pushBackwardIndex);
//            }
//
//        }
//    }







    private boolean gamepad1Dpad(){
        return gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up;
    }

//    private double angle(){
//        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + angleOffset; //(-180,180)
//    }
//    private double angle(){
//        angle =  (rightVerticalPosition - leftVerticalPosition) / 183;
//        angle = angle + 90; // (-90,270)
//        return angle + angleOffset;
//    }
//    private void imuInit(){
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//    }
    private void sleep(int millisecond){
        for(runtime.reset();runtime.milliseconds()<millisecond;){}
    }

    private void autoReturn(){

        if(isReturningToEjectPosition) {

            if (!positionalPID.finish_flag) {
                odometry.updatePositionByStraightPath();
                Position current = new Position(odometry.getRobotX(), odometry.getRobotY(), odometry.angle());
                driveTrain.runToPosition(current,
                        returnPosition,
                        positionalPID.getPositionalPIDOut(current),
                        angularRoughPID.getAngularPIDOut(current.w)
                );

            }

        }

        if(positionalPID.finish_flag){
            isReturningToEjectPosition = false;
        }
    }

    private void positionMove(Position target){

        Position current = odometry.getCurrentPosition();
        driveTrain.runToPosition(current,target,positionalPID.getPositionalPIDOut(current),angularRoughPID.getAngularPIDOut(current.w));


    }

    private void angleCalibration(double target){
        angularAccuratePID.angularPIDUpdate(target);
        while(!angularAccuratePID.finish_flag && runtime.milliseconds()<300){
            odometry.updatePositionByStraightPath();
            driveTrain.move(0,0, angularAccuratePID.getAngularPIDOut(odometry.angle()));

        }
    }
}