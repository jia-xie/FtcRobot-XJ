package org.firstinspires.ftc.teamcode.opmode.otherOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.control.AngularPID;
import org.firstinspires.ftc.teamcode.util.ringDetection.FindRingPipeline;
import org.firstinspires.ftc.teamcode.util.robotAssembly.DriveTrainMotionManagement;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.control.PositionalPID;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.util.MathFunction.getAngle;
import static org.firstinspires.ftc.teamcode.util.MathFunction.getFollowPosition;

@Autonomous
@Config
@Disabled
public class AuxiliaryRed extends LinearOpMode {
    FtcDashboard dashboard;
    OpenCvCamera webcam;
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
    ElapsedTime staticRuntime = new ElapsedTime();
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

    Position targetZonePosition = new Position(11.5,56,-180);
    Position starterStackPosition = new Position (5 * 23 - 11.5,46);
    //PID controller
    AngularPID angularAccuratePID = new AngularPID(0.06, 0, 0.00, 0, 1, 0.03, 0.1, 0);
    AngularPID angularRoughPID = new AngularPID(0.08, 0, 0.00, 0, 1, 0.03, 1, 0);
    PositionalPID positionalPID = new PositionalPID(0.09,0,0,.3,1,0.8,0);
    PositionalPID positionalPurePursuitPID = new PositionalPID(0.07,0,0,.3,1,3,0);
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

    public static Position ejectPosition = new Position(4 * 23 + 11.5,64,10);

    public static double w = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        driveTrain.initDriveTrain(hardwareMap,speedMax);
        initMotorServo();
        odometry = new ThreeWheelOdometryForYuntai(
                motorLeftVertical,motorEjectII_and_RightVertical,motorTakeIn_and_Horizontal,183,
                45 + 3 * 23,9);
        odometry.resetPulseRecord();
        new Thread(odometry).start();




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FindRingPipeline pipeline = new FindRingPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });



        FtcDashboard.getInstance().startCameraStream(webcam,0);

        servoPush.setPosition(pushBackwardIndex);
        servoUpDown.setPosition(upIndex);

        waitForStart();
        staticRuntime.reset();

        boolean signal = false;
        runtime.reset();
        while(opModeIsActive() && runtime.milliseconds()<1000 && !signal) {
            while (!pipeline.finishFlag);
            if(pipeline != null) {
                if (pipeline.getRings() != null) {
                    if (pipeline.getRings().peek() != null) {
                        ringCase = pipeline.getRings().peek().getRingCase();
                        signal = true;
                    }
                }
            }
        }

        switch(ringCase){
            case "Single":
                targetZonePosition.setPosition(23 * 5 + 8,105,90);
                break;
            case "Quad":
                targetZonePosition.setPosition(23 * 5 + 8,111,-180);
                break;
        }
//////////////////////////////////////////////////////////////////////////////////////////////////
        //Launch preloaded rings
        motorControl(2100,0.7);

        positionAccurateMove(2000,new Position(starterStackPosition.x,starterStackPosition.y-10,10),2000,1000);
        sleep(300);
        multiplePush(4,200);
        sleep(300);

        servoUpDown.setPosition(downIndex);

        //Launch StarterStack Rings
        if(ringCase == "Quad"){
            //Round 1
            motorControl(true);
            positionMove(1000, new Position(odometry.getRobotX(),odometry.getRobotY()+10,0),2000);
            positionAccurateMove(1500,new Position(odometry.getRobotX(),odometry.getRobotY(),0),2000,1000);
            sleep(1000);
            motorControl(2200,0.7,false);
            angleCalibration(10,1000);
            sleep(1000);
            loadRing();
            multiplePush(3,0);

            //Round 2
            motorControl(true);
            servoUpDown.setPosition(downIndex);
            positionMove(1000,new Position(odometry.getRobotX(),odometry.getRobotY()+30,0),
                    5000);
            motorControl(2250,1);
            positionAccurateMove(1500,ejectPosition,2000,1000);
            motorControl(false);
            sleep(1000);
            loadRing();
            multiplePush(3,200);
            sleep(300);
            servoUpDown.setPosition(downIndex);

        }else if(ringCase == "Single"){
            motorControl(true);
            positionMove(500,new Position(odometry.getRobotX(),odometry.getRobotY()+15,0),2000);
            motorControl(2350,1,false);
            positionAccurateMove(1500, new Position(34.5,69,10),2000,1000);
            loadRing();
            multiplePush(3,200);
            sleep(300);
            servoUpDown.setPosition(downIndex);
        }

        positionMove(targetZonePosition,5000);
        servoDelivery.setPosition(deliveryDownIndex);
        sleep(400);
        servoPick.setPosition(pickOpenIndex);
        sleep(500);
        servoUpDown.setPosition(downIndex);
        servoDelivery.setPosition(deliveryUpIndex);



        //回城
        Position parkingPosition = new Position (5 * 23 + 11.5,80.5,0);
        if(ringCase == "null"){
            parkingPosition.setPosition(5 * 23 + 11.5,57.5,-90);
        }

        positionAccurateMove(2000,parkingPosition, 5000,5000);

        while(opModeIsActive() && !isStopRequested() && staticRuntime.milliseconds() < 26000);
        if(ringCase == "null"){
            sleep(10000);
            ArrayList<Position> parkingPositions = new ArrayList<>();
            parkingPositions.add(odometry.getCurrentPosition());
            parkingPositions.add(new Position(6 * 23 - 11.5, 2 * 23 + 11.5));
            parkingPositions.add(new Position(6 * 23 - 11.5, 80.5));
            driveTrain.speedIndex = 1500;
            positionalPurePursuitPID.finish_flag = false;

            sleep(10000);
            followPath(parkingPositions,4000);

        }




        odometry.stop();


    }

    //Drive Train
    /**
     * PurePursuit for the pathPositions within timeLimit milliseconds
     * @param pathPositions
     * @param timeLimit
     */
    private void followPath(ArrayList<Position> pathPositions, int timeLimit){
        runtime.reset();
        positionalPurePursuitPID.positionalPIDUpdate(pathPositions.get(pathPositions.size()-1));
        double radius = 5;
        while(opModeIsActive() && runtime.milliseconds()<timeLimit && !positionalPurePursuitPID.finish_flag){

            odometry.updatePositionByStraightPath();
            Position current = odometry.getCurrentPosition();

            Position followPosition = getFollowPosition(pathPositions,current,radius,false,telemetry);
            followPosition.setPosition(followPosition.x,followPosition.y,getAngle(followPosition,current) -90);
            angularRoughPID.angularPIDUpdate(followPosition.w);
            if(radius>0.3 && current.getDistance(pathPositions.get(pathPositions.size()-1)) < radius) {
                radius -=0.3;
            }

            driveTrain.runToPosition(current, followPosition,
                    positionalPurePursuitPID.getPositionalPIDOut(current),
                    angularRoughPID.getAngularPIDOut(current.w));
        }
        driveTrain.stop();
    }
    private void positionAccurateMove(int driveTrainSpeed, Position target, int positionTimeLimit, int angleTimeLimit){
        driveTrain.speedIndex = driveTrainSpeed;
        positionMove(target,positionTimeLimit);
        angleCalibration(target.w,angleTimeLimit);
    }
    private void positionMove(Position target, int timeLimit){
        positionalPID.positionalPIDUpdate(target);
        angularRoughPID.angularPIDUpdate(target.w);
        runtime.reset();
        while(opModeIsActive() && !positionalPID.finish_flag && runtime.milliseconds() < timeLimit) {
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

    private void positionMove(int driveTrainSpeed, Position target, int timeLimit){
        driveTrain.speedIndex = driveTrainSpeed;
        positionMove(target,timeLimit);
    }

    private void angleCalibration(double targetAngle, int timeLimit){

        angularAccuratePID.angularPIDUpdate(targetAngle);
        runtime.reset();
        while(opModeIsActive() && !angularAccuratePID.finish_flag && runtime.milliseconds()<timeLimit){
            driveTrain.move(0,0, angularAccuratePID.getAngularPIDOut(odometry.angle()));
            printNow();
        }
        runtime.reset();

        driveTrain.stop();




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
    private void motorControl(double launchSpeedI, double launchPowerII ){
        motorEjectI.setVelocity(launchSpeedI);
        motorEjectII_and_RightVertical.setPower(launchPowerII);
    }

    private void motorControl(boolean takeInFlag){
        if(takeInFlag) {
            motorTakeIn_and_Horizontal.setPower(-1);
            servoUpDown.setPosition(downIndex);
        }else{
            motorTakeIn_and_Horizontal.setPower(0);
        }
    }

    private void motorControl(double launchSpeedI, double launchPowerII, boolean takeInFlag){
        motorEjectI.setVelocity(launchSpeedI);
        motorEjectII_and_RightVertical.setPower(launchPowerII);
        if(takeInFlag) {
            motorTakeIn_and_Horizontal.setPower(-1);
            servoUpDown.setPosition(downIndex);
        }else{
            motorTakeIn_and_Horizontal.setPower(0);
        }
    }

    private void singlePush(){
        sleep(100);
        servoPush.setPosition(pushForwardIndex);
        sleep(200);
        servoPush.setPosition(pushBackwardIndex);
    }

    private void multiplePush(int pushTime, int timeIntervalAfterEachLaunch){
        for(int i = 0 ; i < pushTime ; i++){
            singlePush();
            sleep(timeIntervalAfterEachLaunch);
        }
    }

    private void loadRing(){
        servoUpDown.setPosition(upIndex);
        sleep(500);
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

}
