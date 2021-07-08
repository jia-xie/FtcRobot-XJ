package org.firstinspires.ftc.teamcode.opmode.otherOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.control.AngularPID;
import org.firstinspires.ftc.teamcode.util.robotAssembly.DriveTrainMotionManagement;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.control.PositionalPID;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;

import java.util.List;

import static org.firstinspires.ftc.teamcode.util.MathFunction.getAngle;
import static org.firstinspires.ftc.teamcode.util.MathFunction.getIntersectionPosition;

@Autonomous
@Config
@Disabled
public class NormalBlue extends LinearOpMode {
    FtcDashboard dashboard;
    public static double X;
    public static double Y;
    public static double W;

    //TensorFlow Lite Model Index
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "ATgiATj/////AAABmRMerr8SOkjHlkK7C7dYREsyziaAdW3bkHelBM9Dis85dmrAm3IorUDfj5eSM1KmU4TBelqnc45mRHjh6SIcbqJq9CXi3Tznux2WXx9ppfDmQR4HHouK5VzkjAWp6ikMCSebOfrlg1jgiMwQYDmsDYJkHlbkoGCMdQdZLpkZ9J609MEDMQhKfR/aVpQdUlU4R+npBAJXymcpRrKJNQLthsVf8rUN6NXWEN/g9zKi64Hlq9uVtcTjcNoppPWii2UimXVA+JKAlaMYJjchM0PxZdYqBhSDpL3xFkceL3kexbg7a63q9KkE371WY4Baf3Rkksxs0cjTFmyENJZh9C/TfrFwbBd07mGDQmuOBCahHfnD";
    private VuforiaLocalizer vuforia;//visual recognition
    private TFObjectDetector tfod;//visual recognition
    private String ringCase = "null";
    private ThreeWheelOdometryForYuntai odometry;

    //
    ElapsedTime runtime = new ElapsedTime();

    DriveTrainMotionManagement driveTrain = new DriveTrainMotionManagement();

    DcMotorEx motorEjectI = null;
    DcMotorEx motorEjectII_and_RightVertical = null;
    DcMotorEx motorLeftVertical = null;
    DcMotorEx motorTakeIn_and_Horizontal = null;

    Servo servoPush = null;
    Servo servoUpDown = null;
    Servo servoDelivery = null;
    Servo servoPick = null;

    public static double angleFollow = 0;

    int speedMax = 1500;

    Position targetZonePosition = new Position(23,80,-90);
    Position starterStackPosition = new Position (1.5 *23,46);
    //PID controller
    AngularPID angularAccuratePID = new AngularPID(0.06, 0, 0.00, 0, 1, 0.03, 0.1, 0);
    AngularPID angularRoughPID = new AngularPID(0.08, 0, 0.00, 0, 1, 0.03, 1, 0);
    PositionalPID positionalPID = new PositionalPID(0.09,0,0,.3,1,0.8,0);
    //Index
    public static double pushForwardIndex = .55;
    public static double pushBackwardIndex = .1;
    public static double upIndex = .65;
    public static double downIndex = .25;
    public static double pickCloseIndex = 1;
    public static double pickHalfCloseIndex = .6;
    public static double pickOpenIndex = .3;
    public static double deliveryDownIndex = .3;
    public static double deliveryUpIndex = 1;

    public static double firstAngle = 0.1;//yuanlibang
    public static double secondAngle = 6;//yuanlibang
    public static double thirdAngle = 11.5;//0.5//yuanlibang
    public static double ejectPowerShotSpeed = 2100;
    public static double ejectPowerShotPower = 0.7;

    public static Position ejectPosition = new Position(34.5,64,10);

    public static double w = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        driveTrain.initDriveTrain(hardwareMap,speedMax);
        initMotorServo();
        odometry = new ThreeWheelOdometryForYuntai(
                motorLeftVertical,motorEjectII_and_RightVertical,motorTakeIn_and_Horizontal,183,
                50,9);
        initVuforia();
        initTfod();


        servoPush.setPosition(pushBackwardIndex);
        servoUpDown.setPosition(upIndex);

        odometry.resetPulseRecord();

        tfod.activate();
        tfod.setZoom(2.5, 16.0 / 9.0);


        waitForStart();



        runtime.reset();
        if (opModeIsActive()) {
            runtime.reset();
            boolean signal = false;
            while (opModeIsActive() && !signal && runtime.milliseconds() < 1000) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        for(Recognition recognition : updatedRecognitions) {

                            ringCase = recognition.getLabel();
                            signal = true;
                        }
//

                        telemetry.update();
                    }
                }
            }
        }
        tfod.shutdown();
//        ringCase = "Quad";

        switch(ringCase){
            case "Single":
                targetZonePosition.setPosition(48,108,-85);
                break;
            case "Quad":
                targetZonePosition.setPosition(27,124,-90);
                break;
        }

        ejectMotor(ejectPowerShotSpeed,ejectPowerShotPower);


        positionMove(new Position(48.5,66.5,firstAngle),5000);

        angleCalibration(firstAngle,3000);
        sleep(200);
        singlePush();
        angleCalibration(secondAngle,3000);
        sleep(200);
        singlePush();
        angleCalibration(thirdAngle,3000);
        sleep(200);
        singlePush();

        driveTrain.speedIndex = 2000;
        switch (ringCase){
            case "null":

                positionMove(new Position(20,125,-70),4000);
                servoUpDown.setPosition(downIndex);
                motorTakeIn_and_Horizontal.setPower(-1);
                driveTrain.speedIndex = 1000;
                positionMove(new Position(23*3,125,-70),3000);
                ejectMotor(2250,.9);
                driveTrain.speedIndex = 2000;
                positionMove(ejectPosition,5000);
                angleCalibration(10,1000);
                motorTakeIn_and_Horizontal.setPower(0);
                sleep(500);
                servoUpDown.setPosition(upIndex);
                sleep(700);
                singlePush();
                singlePush();
                singlePush();
                servoDelivery.setPosition(deliveryUpIndex);
                break;


            case "Single":
                servoUpDown.setPosition(downIndex);
                Position ringTakeInPosition = getIntersectionPosition(
                        odometry.getCurrentPosition(),
                        starterStackPosition,
                        starterStackPosition,
                        15
                ).get(0);
                double angleTakeIn = getAngle(starterStackPosition,odometry.getCurrentPosition())-90;
                ringTakeInPosition.setAngle(angleTakeIn);
                positionMove(ringTakeInPosition,
                        2000);
                motorTakeIn_and_Horizontal.setPower(-0.8);
                starterStackPosition.setAngle(angleTakeIn);
                ejectMotor(2250,.9);
                positionMove(starterStackPosition,3000);
                positionMove(ejectPosition,5000);
                angleCalibration(10,1000);
                motorTakeIn_and_Horizontal.setPower(0);
                sleep(500);
                servoUpDown.setPosition(upIndex);
                sleep(700);
                singlePush();

                servoUpDown.setPosition(downIndex);
                ejectMotor(2250,.9);
                motorTakeIn_and_Horizontal.setPower(-1);
                positionMove(new Position(15,124,0),3000);
                positionMove(new Position(60,124,-70),3000);
                positionMove(ejectPosition,5000);
                angleCalibration(10,1000);
                motorTakeIn_and_Horizontal.setPower(0);
                sleep(500);
                servoUpDown.setPosition(upIndex);
                sleep(700);
                singlePush();
                sleep(100);
                singlePush();
                sleep(100);
                singlePush();
                sleep(100);
                singlePush();
                sleep(100);
//                servoDelivery.setPosition(deliveryUpIndex);
                break;

            case "Quad":

                servoUpDown.setPosition(downIndex);

                double x = 35;
                double y = 51.5;

//                double x = 96.5+2;
//                double y = 53;
                positionMove(new Position(x + 10,y + 10,135),2000);
                angleCalibration(135,1000);

                driveTrain.speedIndex =2000;
                motorTakeIn_and_Horizontal.setPower(0.3);
                positionMove(new Position(x , y , 135), 2000);

                driveTrain.speedIndex =1000;
                motorTakeIn_and_Horizontal.setPower(-0.6);
                positionMove(new Position(x - 2, y - 2, 135), 3000);

                driveTrain.speedIndex =2000;
                motorTakeIn_and_Horizontal.setPower(0.5);
                positionMove(new Position(x - 1, y - 1, 135), 1000);

                driveTrain.speedIndex =1500;
                motorTakeIn_and_Horizontal.setPower(-1);
                positionMove(new Position(x - 4, y - 4, 135), 3000);

                driveTrain.speedIndex =2000;

                ejectMotor(2250,.9);
                positionMove(ejectPosition,5000);
                angleCalibration(10,1000);
                motorTakeIn_and_Horizontal.setPower(0);
                sleep(200);
                servoUpDown.setPosition(upIndex);
                sleep(300);
                singlePush();
                sleep(100);
                singlePush();
                sleep(100);
                singlePush();
                sleep(300);

                servoUpDown.setPosition(downIndex);
                motorTakeIn_and_Horizontal.setPower(-1);
                positionMove(new Position(x - 4,y - 4,135),2000);
                angleCalibration(135,1000);

                driveTrain.speedIndex =1000;
                motorTakeIn_and_Horizontal.setPower(-0.6);
                positionMove(new Position(x - 8, y - 8, 135), 3000);

                driveTrain.speedIndex =2000;
                motorTakeIn_and_Horizontal.setPower(0.5);
                positionMove(new Position(x - 6, y - 6, 135), 1000);

                driveTrain.speedIndex =1500;
                motorTakeIn_and_Horizontal.setPower(-1);
                positionMove(new Position(x - 12, y - 12, 135), 3000);

                driveTrain.speedIndex =2000;

                positionMove(ejectPosition,5000);
                angleCalibration(10,1000);
                motorTakeIn_and_Horizontal.setPower(0);
                sleep(200);
                servoUpDown.setPosition(upIndex);
                sleep(300);
                singlePush();
                sleep(100);
                singlePush();
                sleep(100);
                singlePush();
                sleep(300);


                break;
//
//                for (int i = 0; i < 2; i++) {
//                    x = odometry.getRobotX();
//                    y = odometry.getRobotY();
//                    positionMove(new Position(x + i + 1, y + i + 1, 135), 2000);
//                    if (i > 0) {
//                        motorTakeIn_and_Horizontal.setPower(-1);
//                        sleep(300);
//                    }
//                    motorTakeIn_and_Horizontal.setPower(0.5);
//                    sleep(100);
//                    positionMove(new Position(x - 3, y - 3, 135), 2000);
//                }
//                ejectMotor(2250,.9);
//                positionMove(ejectPosition,5000);
//                angleCalibration(10,1000);
//                motorTakeIn_and_Horizontal.setPower(0);
//                sleep(100);
//                servoUpDown.setPosition(upIndex);
//                sleep(700);
//                singlePush();
//                sleep(100);
//                singlePush();
//                sleep(100);
//                singlePush();
//                sleep(300);
//                x = 32.5;
//                y = 44;
//                servoUpDown.setPosition(downIndex);
//                positionMove(new Position(x,y,135),3000);
//                motorTakeIn_and_Horizontal.setPower(.9);
//                sleep(300);
//                motorTakeIn_and_Horizontal.setPower(0);
//                for (int i = 0; i < 2; i++) {
//                    x = odometry.getRobotX();
//                    y = odometry.getRobotY();
//                    positionMove(new Position(x + 2, y + 2, 135), 2000);
//                    if (i > 0) {
//                        motorTakeIn_and_Horizontal.setPower(0.5);
//                        sleep(300);
//                    }
//                    motorTakeIn_and_Horizontal.setPower(-1);
//                    sleep(100);
//                    positionMove(new Position(x - 3, y - 3, 135), 2000);
//                }
//                positionMove(ejectPosition,5000);
//                angleCalibration(10,1000);
//                motorTakeIn_and_Horizontal.setPower(0);
//                sleep(100);
//                servoUpDown.setPosition(upIndex);
//                sleep(700);
//                singlePush();
//                sleep(100);
//                singlePush();
//                sleep(100);
//                singlePush();
//                servoDelivery.setPosition(deliveryUpIndex);
//
//                break;
        }
        //first wobble goal
        positionMove(targetZonePosition,5000);
        servoDelivery.setPosition(deliveryDownIndex);
        sleep(400);
        servoPick.setPosition(pickOpenIndex);
        sleep(500);
        servoUpDown.setPosition(downIndex);

        //second wobble goal
//        positionalPID = new PositionalPID(0.05,0,0,.5,1,1,0);
//        positionMove(new Position(120,36,0),5000);
//
//        servoPick.setPosition(pickCloseIndex);
//        sleep(600);
//        servoDelivery.setPosition(deliveryUpIndex);
//
//        positionMove(targetZonePosition,3000);
//        servoPick.setPosition(pickOpenIndex);
//        servoDelivery.setPosition(deliveryDownIndex);
//        sleep(500);

        //回城
        Position parkingPosition = new Position (25,85,0);
        if(ringCase == "null"){
            parkingPosition.setPosition(34.5,85,0);
        }

        positionMove(parkingPosition, 1000000);





        while(opModeIsActive() && !isStopRequested()){ }
    }

    //Drive Train
    private void positionMove(Position target, double timeLimit){
        positionalPID.positionalPIDUpdate(target);
        angularRoughPID.angularPIDUpdate(target.w);
        runtime.reset();
        while(opModeIsActive() && !positionalPID.finish_flag && runtime.milliseconds() < timeLimit) {
            odometry.updatePositionByStraightPath();
            Position current = new Position(odometry.getRobotX(), odometry.getRobotY(), odometry.angle());
            driveTrain.runToPosition(current,
                    target,
                    positionalPID.getPositionalPIDOut(current),
                    angularRoughPID.getAngularPIDOut(current.w)
            );
            printNow();
        }
        driveTrain.stop();

    }

    private void angleCalibration(double targetAngle, int timeLimit){

        angularAccuratePID.angularPIDUpdate(targetAngle);
        runtime.reset();
        while(opModeIsActive() && !angularAccuratePID.finish_flag && runtime.milliseconds()<timeLimit){
            odometry.updatePositionByStraightPath();
            driveTrain.move(0,0, angularAccuratePID.getAngularPIDOut(odometry.angle()));
            printNow();
        }
        runtime.reset();

            driveTrain.stop();
            odometry.updatePositionByStraightPath();



    }
    private void printNow(){

//        TelemetryPacket xPosPacket = new TelemetryPacket();
//        TelemetryPacket yPosPacket = new TelemetryPacket();
//        TelemetryPacket wPosPacket = new TelemetryPacket();
//        xPosPacket.put("x",odometry.getRobotX());
//        yPosPacket.put("y",odometry.getRobotY());
//        wPosPacket.put("w",odometry.angle());
//        dashboard.sendTelemetryPacket(xPosPacket);
//        dashboard.sendTelemetryPacket(yPosPacket);
//        dashboard.sendTelemetryPacket(wPosPacket);
        TelemetryPacket packet = new TelemetryPacket();
        w = odometry.angle();
        packet.put("w",w);
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Ring Label", ringCase);
        telemetry.addData("X",odometry.getRobotX());
        telemetry.addData("Y",odometry.getRobotY());
        telemetry.addData("W",odometry.angle());
        telemetry.update();
    }
    //Ring
    private void ejectMotor(double speedI, double powerII ){
        motorEjectI.setVelocity(speedI);
        motorEjectII_and_RightVertical.setPower(powerII);
    }

    private void singlePush(){
        sleep(100);
        servoPush.setPosition(pushForwardIndex);
        sleep(200);
        servoPush.setPosition(pushBackwardIndex);
    }

    private void initMotorServo(){
        motorEjectI = hardwareMap.get(DcMotorEx.class,"m20");
        motorEjectII_and_RightVertical = hardwareMap.get(DcMotorEx.class, "m21");//EjectII
        motorLeftVertical = hardwareMap.get(DcMotorEx.class, "m23");
        motorTakeIn_and_Horizontal = hardwareMap.get(DcMotorEx.class,"m22");

        servoPush = hardwareMap.get(Servo.class,"sP");
        servoUpDown = hardwareMap.get(Servo.class,"sU");
        servoDelivery = hardwareMap.get(Servo.class, "sD");
        servoPick = hardwareMap.get(Servo.class, "sPick");
    }
    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
