package org.firstinspires.ftc.teamcode.opmode.otherOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.control.AngularPID;
import org.firstinspires.ftc.teamcode.util.robotAssembly.DriveTrainMotionManagement;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.control.PositionalPID;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;

import static java.lang.Math.abs;

@Config
@Disabled
@TeleOp(group = "OpMode",name = "Volt OpMode")
public class Volt extends OpMode {

    FtcDashboard dashboard;

    public static int interval = 150;

    //Parameters
    double speedIndexR = 0;
    int speedMax = 2000;
    double x = 0;
    double y = 0;
    double w = 0;

    ThreeWheelOdometryForYuntai odometry;

    private final double wheelDiameter = 2;     //inch
    private final double ppr = 4096;
    private double inPerEncoder = ((wheelDiameter * Math.PI) / ppr);

    ElapsedTime runtime = new ElapsedTime();
    public static double finish_range = 400;
    public static double p = 0.0001;
    public static double i = 0;
    public static double d = 0;
    public static double initialDistance = 25000;
    public static double distance = 3000;


    //Servo Index
    public static double pushForwardIndex = .5;
    public static double pushBackwardIndex = .1;
    public static double upIndex = .6;
    public static double downIndex = .31;
    public static double pickCloseIndex = 1;
    public static double pickHalfCloseIndex = .6;
    public static double pickOpenIndex = 0;
    public static double deliveryDownIndex = .3;
    public static double deliveryUpIndex = 1;

    public static double speedEject = 0;
    public static double highGoalEjectPower = 1;
    //Motor Index
    public static double highGoalEjectSpeed = 2350;
    public static double powerEjectSpeedI = 2100;

    AngularPID angularRoughPID = new AngularPID(0.08, 0, 0.00, 0, 1, 0.03, 0.5, 0);
    PositionalPID positionalPID = new PositionalPID(0.09,0,0,.3,1,0.8,0);
    Position returnPosition = new Position(100.5,90,10);

    double offset = 0;
    //imu:
    BNO055IMU imu;

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

    boolean isEjectingSingle = false;
    boolean isEjectingTriple = false;
    boolean isEjectingPowerShotStage = false;
    boolean isEjectingPowerShotStageA = false;
    boolean isEjectingPowerShotStageB = false;
    boolean isEjectingPowerShotStageC = false;
    boolean isRuntimeReset = false;
    boolean toBeReset = false;
    boolean isReturningToEjectPosition = false;
    boolean isRecordingCurrentPosition = false;

    //PID Controllers
    AngularPID angularPID = new AngularPID(0.04, 0, 0.003, 0, 1, 0.1, 1, 0);

    double leftVerticalPosition;
    double rightVerticalPosition;
    Position powerShotAuxiliaryPosition = new Position (112,75,25);

    TelemetryPacket packet = new TelemetryPacket();
    double targetAngle = 10;
    @Override
    public void init() {
//        dashboard =  FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());


        imuInit();
        driveTrain.initDriveTrain(hardwareMap,speedMax);


        motorEjectI = hardwareMap.get(DcMotorEx.class,"m20");
        motorEjectII_and_RightVertical = hardwareMap.get(DcMotorEx.class, "m21");
        motorLeftVertical = hardwareMap.get(DcMotorEx.class, "m23");
        motorTakeIn_and_Horizontal = hardwareMap.get(DcMotorEx.class,"m22");

        odometry = new ThreeWheelOdometryForYuntai(
                motorLeftVertical, motorEjectII_and_RightVertical, motorTakeIn_and_Horizontal,
                183,0, 0);

        odometry.resetPulseRecord();

        servoPush = hardwareMap.get(Servo.class,"sP");
        servoUpDown = hardwareMap.get(Servo.class,"sU");
        servoDelivery = hardwareMap.get(Servo.class, "sD");
        servoPick = hardwareMap.get(Servo.class, "sPick");

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
        if(angularPID.finish_flag && positionalPID.finish_flag) driveTrainMove();
//        autoPositioning();

        //Motor Control
        motorMove();

        //Servo Control
        servoMove();

        //Auxiliary Auto Control
        recordCurrentPosition();
        tripleEject(interval);
        autoEjectPowerShot();
        autoReturn();



        speedFollow = motorEjectI.getVelocity();

//        telemetry.addData("Speed Follow", speedFollow);
        speedEject = motorEjectI.getVelocity();

        telemetry.addData("X",odometry.getRobotX());
        telemetry.addData("Y",odometry.getRobotY());
//        telemetry.addData("speedI",speedEject);
//        telemetry.addData("Power",motorEjectI.getPower());
//        telemetry.addData("Right", odometry.getYRightPulse());
//        telemetry.addData("Left", odometry.getYLeftPulse());
//        telemetry.addData("Horizontal", odometry.getXPulse());
        telemetry.addData("Angle",odometry.angle());

        telemetry.addData("ALL",isEjectingPowerShotStage);
        telemetry.addData("A",isEjectingPowerShotStageA);
        telemetry.addData("B",isEjectingPowerShotStageB);
        telemetry.addData("C",isEjectingPowerShotStageC);

        telemetry.addData("pos",positionalPID.finish_flag);
        telemetry.addData("ang",angularRoughPID.finish_flag);

        telemetry.update();
    }

    private void controllerUpdate(){

        if(!gamepad1.atRest() || gamepad1Dpad()){
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            w = gamepad1.right_stick_x;
            speedIndexR = gamepad1.right_trigger;
            isEjectingTriple = false;
            isReturningToEjectPosition = false;
            positionalPID.finish_flag = true;
            positionalPID.finish_flag = true;
            angularPID.finish_flag = true;
            isReturningToEjectPosition = false;
            isEjectingTriple = false;

        }else{
            x = gamepad2.left_stick_x;
            y = -gamepad2.left_stick_y;
            w = gamepad2.right_stick_x;
            speedIndexR = gamepad2.right_trigger;
        }

        if(gamepad1.a){
            positionalPID.finish_flag = false;
            isReturningToEjectPosition = true;
        }

        if(gamepad1.b){
            isEjectingTriple = true;
            runtime.reset();
        }
        if(gamepad2.right_trigger > 0.5){
            positionalPID.positionalPIDUpdate(odometry.getCurrentPosition());
            angularRoughPID.angularPIDUpdate(odometry.angle());
            returnPosition.setPosition(odometry.getCurrentPosition());
        }
        if(gamepad1.right_trigger > 0.5 && gamepad1.left_trigger > 0.5){
            isEjectingPowerShotStage = true;
            isEjectingPowerShotStageA = true;
            isEjectingPowerShotStageB = false;
            isEjectingPowerShotStageC = false;
            positionalPID.positionalPIDUpdate(powerShotAuxiliaryPosition);
            angularPID.angularPIDUpdate(powerShotAuxiliaryPosition.w);
            isRuntimeReset = false;
            isEjectingSingle = true;
        }
        if(gamepad2.left_trigger > 0.5){
            isRecordingCurrentPosition = true;
        }
        if(gamepad2.y){
            resetHorizontalEncoderWheel();
        }

        if(gamepad1Dpad()){
            if(gamepad1.dpad_left){
                w = -0.2;
            }else if(gamepad1.dpad_right){
                w = 0.2;
            }else if(gamepad1.dpad_down){
                y = -0.2;
            }else if(gamepad1.dpad_up){
                y = 0.2;
            }
        }



        //        if(!(gamepad1.atRest() || gamepad1Dpad() || gamepad1.a|| gamepad1.x || gamepad1.y)){
//            isEjectingTriple = false;
//        }
//
//        if(!(gamepad1.atRest() || gamepad1Dpad() || gamepad1.a|| gamepad1.b || gamepad1.x || gamepad1.y)){
//            isEjectingTriple = false;
//        }

    }

    private void driveTrainMove(){
        if(angularPID.finish_flag && !isEjectingPowerShotStage) {
            if (abs(x) > 0.01 || abs(y) > 0.01 || abs(w) > 0.01) {
                if (speedIndexR > 0.05) {
                    driveTrain.move(x * (1 - speedIndexR * .8), y * (1 - speedIndexR * .8), w * (1 - speedIndexR * .8));
                } else {
                    driveTrain.move(x, y, w);
                }
            } else {
                driveTrain.stop();
            }
        }
    }

    private void motorMove(){
        if(gamepad1.right_stick_button){
            motorEjectI.setVelocity(highGoalEjectSpeed);
//            motorEjectI.setPower(1);
            motorEjectII_and_RightVertical.setPower(highGoalEjectPower);

        }else if(gamepad1.left_stick_button){
            motorEjectI.setPower(0);
            motorEjectII_and_RightVertical.setPower(0);
        }else if(gamepad1.left_trigger>0.5){
            motorEjectI.setVelocity(powerEjectSpeedI);
            motorEjectII_and_RightVertical.setPower(.7);
        }
        //Take In
        if(gamepad1.right_bumper){
            motorTakeIn_and_Horizontal.setPower(-0.7);
        }else if(gamepad1.left_bumper){
            motorTakeIn_and_Horizontal.setPower(0.7);
        }else{
            motorTakeIn_and_Horizontal.setPower(0);
        }
    }

    private void servoMove(){
        //Push
        if(gamepad1.x){
            servoPush.setPosition(pushForwardIndex);
        }else{
            servoPush.setPosition(pushBackwardIndex);
        }

        //UpDown
        if(gamepad1.y){
            servoUpDown.setPosition(upIndex);
        }

        //gamepad 1 down
        if(gamepad1.right_bumper){
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
        if(isEjectingSingle) {
            if(runtime.milliseconds() < 100) {
                servoPush.setPosition(pushForwardIndex);
            }else if(runtime.milliseconds() < 250){
                servoPush.setPosition(pushBackwardIndex);
                isEjectingSingle = false;
            }
        }
    }


    private void tripleEject(int timeInterval){
        if(isEjectingTriple || !gamepad1.b) {
            if(runtime.milliseconds() < 150){
                servoPush.setPosition(pushForwardIndex);
            }else if(runtime.milliseconds() < 300){
                servoPush.setPosition(pushBackwardIndex);
            }

            else if(runtime.milliseconds()<450){
                servoPush.setPosition(pushForwardIndex);
            }else if(runtime.milliseconds()<600){
                servoPush.setPosition(pushBackwardIndex);
            }

            else if(runtime.milliseconds() < 750) {
                servoPush.setPosition(pushForwardIndex);
            }
            else if(runtime.milliseconds() < 900) {
                servoPush.setPosition(pushBackwardIndex);
            }

        }
    }

    private void autoEjectPowerShot(){
        if(isEjectingPowerShotStage) {
            if (isEjectingPowerShotStageA) {
                ejectPowerShotStageA();
            } else if (isEjectingPowerShotStageB) {
                ejectPowerShotStageB();
            } else if (isEjectingPowerShotStageC) {
                ejectPowerShotStageC();
            }
        }

    }

    private void recordCurrentPosition(){
        if(isRecordingCurrentPosition){
            powerShotAuxiliaryPosition = new Position(
                    odometry.getRobotX()-15,
                    odometry.getRobotY()-7,
                    odometry.angle()+25);
            isRecordingCurrentPosition = false;
            positionalPID.positionalPIDUpdate(powerShotAuxiliaryPosition);
            angularPID.angularPIDUpdateOnlyForTheSigma(powerShotAuxiliaryPosition.w);
        }

    }

    private void ejectPowerShotStageA(){
        if(!positionalPID.finish_flag){
            positionMove(powerShotAuxiliaryPosition);
        }else if(isRuntimeReset){
            runtime.reset();
            isRuntimeReset = false;
        }else if(isEjectingSingle){
            ejectSingle();
        }else{
            isEjectingPowerShotStageA = false;
            isEjectingPowerShotStageB = true;
            isRuntimeReset = true;
            isEjectingSingle = true;
            powerShotAuxiliaryPosition.setPosition(
                    powerShotAuxiliaryPosition.x-15,
                    powerShotAuxiliaryPosition.y-5,
                    powerShotAuxiliaryPosition.w+5);
            positionalPID.positionalPIDUpdate(powerShotAuxiliaryPosition);
            angularPID.angularPIDUpdate(powerShotAuxiliaryPosition.w);
        }

    }

    private void ejectPowerShotStageB(){
        if(!positionalPID.finish_flag){
            positionMove(powerShotAuxiliaryPosition);
        }else if(isRuntimeReset){
            runtime.reset();
            isRuntimeReset = false;
        }else if(isEjectingSingle){
            ejectSingle();
        } else{
            isEjectingPowerShotStageB = false;
            isEjectingPowerShotStageC = true;
            isRuntimeReset = true;
            isEjectingSingle = true;
            powerShotAuxiliaryPosition.setPosition(
                    powerShotAuxiliaryPosition.x,
                    powerShotAuxiliaryPosition.y,
                    powerShotAuxiliaryPosition.w+5);
            positionalPID.positionalPIDUpdate(powerShotAuxiliaryPosition);
        }
    }

    private void ejectPowerShotStageC(){
        if(!positionalPID.finish_flag){
            positionMove(powerShotAuxiliaryPosition);
        }else if(isRuntimeReset){
            runtime.reset();
            isRuntimeReset = false;
        }else if(isEjectingSingle){
            ejectSingle();
        } else{
            isEjectingPowerShotStageC = false;
            isEjectingPowerShotStage = false;
            isRuntimeReset = true;
            isEjectingSingle = true;
            powerShotAuxiliaryPosition.setPosition(
                    powerShotAuxiliaryPosition.x,
                    powerShotAuxiliaryPosition.y,
                    powerShotAuxiliaryPosition.w+5);
            positionalPID.positionalPIDUpdate(powerShotAuxiliaryPosition);
        }
    }

    //Other Functions
    public void resetHorizontalEncoderWheel(){
        offset = -motorTakeIn_and_Horizontal.getCurrentPosition();
    }

    public double getXWheelEncoder(){
        return motorTakeIn_and_Horizontal.getCurrentPosition() + offset;
    }

    public void resetRuntime(){
        runtime.reset();
        isRuntimeReset = true;
    }

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
    private void imuInit(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
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
}
