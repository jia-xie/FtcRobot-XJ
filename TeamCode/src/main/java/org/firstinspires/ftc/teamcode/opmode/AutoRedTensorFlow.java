package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.util.MathFunction.getAngle;
import static org.firstinspires.ftc.teamcode.util.MathFunction.getFollowPosition;

@Autonomous
@Config
public class AutoRedTensorFlow extends LinearOpMode {
    FtcDashboard dashboard;
    public static double X;
    public static double Y;
    public static double W;

    //TensorFlow Lite Model Index
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "ATgiATj/////AAABmRMerr8SOkjHlkK7C7dYREsyziaAdW3bkHelBM9Dis85dmrAm3IorUDfj5eSM1KmU4TBelqnc45mRHjh6SIcbqJq9CXi3Tznux2WXx9ppfDmQR4HHouK5VzkjAWp6ikMCSebOfrlg1jgiMwQYDmsDYJkHlbkoGCMdQdZLpkZ9J609MEDMQhKfR/aVpQdUlU4R+npBAJXymcpRrKJNQLthsVf8rUN6NXWEN/g9zKi64Hlq9uVtcTjcNoppPWii2UimXVA+JKAlaMYJjchM0PxZdYqBhSDpL3xFkceL3kexbg7a63q9KkE371WY4Baf3Rkksxs0cjTFmyENJZh9C/TfrFwbBd07mGDQmuOBCahHfnD";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

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

    Position targetZonePosition = new Position(5 * 23 - 7,4 * 23 - 4,90);
    Position starterStackPosition = new Position (4.5 * 23,46 + 2);
    Position highGoalLaunchPositionI = new Position(90,67,4);

    //PID controller
    AngularPID angularAccuratePID = new AngularPID(0.06, 0, 0.00, 0, 1, 0.03, 0.1, 0);
    AngularPID angularRoughPID = new AngularPID(0.08, 0, 0.00, 0, 1, 0.03, 1, 0);
    PositionalPID positionalPID = new PositionalPID(0.07,0,0,.3,1,0.8,0);
    PositionalPID positionalPurePursuitPID = new PositionalPID(0.06,0,0,.3,1,2,0);
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

    public static Position ejectPosition = new Position(4 * 23 + 11.5,69,12);

    public static double highGoalEjectPower = 1;
    //Motor Index
    public static double highGoalEjectSpeed = 2350;

    public static double w = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Not ready for start");
        telemetry.update();
        dashboard = FtcDashboard.getInstance();
        driveTrain.initDriveTrain(hardwareMap,speedMax);
        initMotorServo();
        odometry = new ThreeWheelOdometryForYuntai(
                motorLeftVertical,motorEjectII_and_RightVertical,motorTakeIn_and_Horizontal,183,
                3* 23 + 26 - 4.33,9);



        odometry.resetPulseRecord();
        new Thread(odometry).start();

        initVuforia();
        initTfod();

        servoPush.setPosition(pushBackwardIndex);
        servoUpDown.setPosition(upIndex);
        telemetry.addLine("Ready for start");
        telemetry.update();


        tfod.activate();
        tfod.setZoom(2.5, 16.0 / 9.0);
        FtcDashboard.getInstance().startCameraStream(tfod,0);

        waitForStart();

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
                        for (Recognition recognition : updatedRecognitions) {

                            ringCase = recognition.getLabel();
                            signal = true;
//
                        }
                        telemetry.update();
                    }
                }
            }
        }
        tfod.deactivate();



        switch(ringCase){
            case "Single":
                targetZonePosition.setPosition(4 * 23 - 3,5 * 23 - 4,90);
                break;
            case "Quad":
                targetZonePosition.setPosition(5 * 23 - 5,6 * 23 - 9,135);
                break;
        }


        motorControl(highGoalEjectSpeed, highGoalEjectPower);
        ArrayList<Position> highGoalPathPositions = new ArrayList<>();
        highGoalPathPositions.add(odometry.getCurrentPosition());
        highGoalPathPositions.add(starterStackPosition.getExtendedPosition(-20,0));
        highGoalPathPositions.add(highGoalLaunchPositionI);
        followPathWithoutTurningTowards(2000,9,highGoalPathPositions,10, 5000,false,4);
        positionAccurateMove(2000, highGoalLaunchPositionI,4500,2000);
        multiplePush(2,400);
        singlePush();

        servoUpDown.setPosition(downIndex);
        if(ringCase != "null") {
            switch (ringCase) {
                case "Single":
                    motorControl(true);
                    servoUpDown.setPosition(downIndex);

                    double angleTakeInSingle = getAngle(starterStackPosition, odometry.getCurrentPosition()) - 90;
                    angleCalibration(angleTakeInSingle,2000);
                    starterStackPosition.setAngle(angleTakeInSingle);
                    motorControl(2350, 1);
                    positionMove(starterStackPosition, 3000);
                    positionAccurateMove(1500, highGoalLaunchPositionI, 5000, 1000);
                    motorTakeIn_and_Horizontal.setPower(0);
                    sleep(500);
                    loadRing();
                    multiplePush(3, 0);

                    break;

                case "Quad":
                    ArrayList<Position> quadLaunchPositions = new ArrayList<>();
                    quadLaunchPositions.add(odometry.getCurrentPosition());
                    quadLaunchPositions.add(starterStackPosition.getExtendedPosition(-20, 0));
                    quadLaunchPositions.add(starterStackPosition.getExtendedPosition(-15, -12));
                    quadLaunchPositions.add(starterStackPosition.getExtendedPosition(0, -12));
                    followPathWithoutTurningTowards(1500,4, quadLaunchPositions, 0, 3000, true,4);
                    positionAccurateMove(2000,quadLaunchPositions.get(quadLaunchPositions.size()-1),2000,1000);

                    //Round 1
                    motorControl(2350, 1, true);
                    positionMove(800, new Position(starterStackPosition.x, starterStackPosition.y - 3,0), 2000);
                    motorControl(false);
                    positionMove(800, new Position(odometry.getRobotX(), odometry.getRobotY() + 21), 2000);
                    positionAccurateMove(1500, new Position(odometry.getRobotX(), odometry.getRobotY() -3, 13), 2000, 1000);
                    motorControl(true);
                    sleep(1500);
                    motorControl(false);
                    loadRing();
                    multiplePush(3, 400);

                    //Round 2
                    motorControl(true);
                    servoUpDown.setPosition(downIndex);
                    positionMove(1000, new Position(odometry.getRobotX(), odometry.getRobotY() + 3, 0),
                            5000);
                    motorControl(2350, 1);
                    positionAccurateMove(1500, ejectPosition, 2000, 1000);
                    sleep(1000);
                    motorControl(false);
                    loadRing();
                    multiplePush(2, 400);
                    sleep(300);
                    servoUpDown.setPosition(downIndex);

                    break;
            }
        }
//        else {
//
//
//            positionAccurateMove(2000, new Position(3 * 23 + 7, 2 * 23 + 11.5, -20), 3000, 1000);
//            telemetry.addLine("Start Processing Ring Position");
//            telemetry.update();
//            runtime.reset();
//            ArrayList<Position> ringCollectPathPositions = new ArrayList<>();
//            Position current = odometry.getCurrentPosition();
//            ringCollectPathPositions.add(current);
//            boolean isPathCollected = false;
//            while (opModeIsActive() && !isStopRequested() && !isPathCollected && runtime.milliseconds() < 3000) {
////                while (!pipeline.finishFlag) ;
////                boolean isDetectedThreePositions = false;
////                for (Ring ring : pipeline.getRings()) {
////                    if (!isDetectedThreePositions) {
////                        Position ringPos = ring.getPosition();
////                        ringPos = current.getExtendedPosition(ringPos);
////                        ringPos.setRelativeX(-20);
////                        ringCollectPathPositions.add(ringPos);
////                        telemetry.addLine(ringCollectPathPositions.get(ringCollectPathPositions.size() - 1).toString());
////                    }
////                    if (ringCollectPathPositions.size() > 3) isDetectedThreePositions = true;
////                }
////                ringCollectPathPositions = pipeline.getPositionsOfRingsInRange((int)130,240, odometry.getCurrentPosition());
//                isPathCollected = true;
//                Thread.yield();
//            }
//
//
//            telemetry.addLine("Processing Completed");
//            telemetry.update();
//
//            if(ringCollectPathPositions.size() > 4) {
//                for(int i = 3; i < ringCollectPathPositions.size() ; i++) ringCollectPathPositions.remove(i);
//            }
//            telemetry.update();
//
//
//            motorControl(true);
//            driveTrain.MAX_SPEED = 1000;
//            if(ringCollectPathPositions.size() > 1) {
//                followPath(1, ringCollectPathPositions, 5000);
//
//                motorControl(2350, 1);
//                positionAccurateMove(2000, ejectPosition, 2000, 1000);
//                motorControl(false);
//                loadRing();
//                sleep(300);
//                multiplePush(4, 0);
//            }
//        }

        positionMove(2000,targetZonePosition,5000);
        servoDelivery.setPosition(deliveryDownIndex);
        sleep(400);
        servoPick.setPosition(pickOpenIndex);
        sleep(500);
        servoDelivery.setPosition(deliveryUpIndex);




        //Parking
        if(ringCase == "null"){
            positionAccurateMove(2000,new Position(4 * 23 - 11.5,80.5,0),3000,2000);
        }else {
            ArrayList<Position> parkingPathPositions = new ArrayList<>();
            parkingPathPositions.add(odometry.getCurrentPosition());
            switch (ringCase) {
                case "Single":
                    parkingPathPositions.add(new Position(3 * 23 + 11.5, 5 * 23 - 11));
                    break;
                case "Quad":
                    parkingPathPositions.add(new Position(3 * 23 + 11.5, 6 * 23 - 11));
                    break;
            }
            parkingPathPositions.add(new Position(3 * 23 + 11.5,80.5));

            followPathWithoutTurningTowards(2000,9,parkingPathPositions,0,3000,false,4);
//            positionAccurateMove(2000,new Position(57.5,80.5,0),3000,1000);
            positionAccurateMove(2000,new Position(3 * 23 + 11.5,82.5,0),3000,1000);
        }

        servoUpDown.setPosition(downIndex);
        odometry.stop();

    }

    //Drive Train

    /**
     * PurePursuit for the pathPositions within timeLimit milliseconds
     * @param pathPositions
     * @param timeLimit
     */
    private void followPath(double radius, ArrayList<Position> pathPositions, int timeLimit){
        positionalPurePursuitPID.positionalPIDUpdate(pathPositions.get(pathPositions.size()-1));

        while(opModeIsActive() && runtime.milliseconds()<timeLimit && !positionalPurePursuitPID.finish_flag){

            odometry.updatePositionByStraightPath();
            Position current = odometry.getCurrentPosition();

            Position followPosition = getFollowPosition(pathPositions,current,radius,false,telemetry);
            followPosition.setPosition(followPosition.x,followPosition.y,getAngle(followPosition,current) -90);
            angularRoughPID.angularPIDUpdate(followPosition.w);
            if(radius>1 && current.getDistance(pathPositions.get(pathPositions.size()-1)) < radius + 3) {
                radius--;
            }
            driveTrain.runToPosition(current, followPosition,
                    positionalPurePursuitPID.getPositionalPIDOut(current),
                    angularRoughPID.getAngularPIDOut(current.w));
        }
        driveTrain.stop();
    }

    private void followPathWithoutTurningTowards(int speedMax, double radius, ArrayList<Position> pathPositions, double desiredAngle, int timeLimit, boolean reverseFlag, double finishRange){
        driveTrain.speedIndex = speedMax;
        positionalPurePursuitPID.positionalPIDUpdate(pathPositions.get(pathPositions.size()-1));
        positionalPurePursuitPID.setFinishRange(finishRange);
        while(opModeIsActive() && runtime.milliseconds()<timeLimit && !positionalPurePursuitPID.finish_flag){

            odometry.updatePositionByStraightPath();
            Position current = odometry.getCurrentPosition();

            Position followPosition = getFollowPosition(pathPositions,current,radius,reverseFlag,telemetry);
            followPosition.setPosition(followPosition.x,followPosition.y,desiredAngle);
            angularRoughPID.angularPIDUpdate(followPosition.w);
            if(radius>1 && current.getDistance(pathPositions.get(pathPositions.size()-1)) < radius + 3) {
                radius--;
            }
            driveTrain.runToPosition(current, followPosition,
                    positionalPurePursuitPID.getPositionalPIDOut(current),
                    angularRoughPID.getAngularPIDOut(current.w));
        }
        driveTrain.stop();
    }

    /**
     * PurePursuit for the pathPositions within timeLimit milliseconds
     * @param pathPositions
     * @param timeLimit
     */
    private void followPath(int speedMax, int decrementRadius, double radius, ArrayList<Position> pathPositions, int timeLimit){
        driveTrain.speedIndex = speedMax;
        positionalPurePursuitPID.positionalPIDUpdate(pathPositions.get(pathPositions.size()-1));

        while(opModeIsActive() && runtime.milliseconds()<timeLimit && !positionalPurePursuitPID.finish_flag){

            odometry.updatePositionByStraightPath();
            Position current = odometry.getCurrentPosition();

            Position followPosition = getFollowPosition(pathPositions,current,radius,false,telemetry);
            followPosition.setPosition(followPosition.x,followPosition.y,getAngle(followPosition,current) -90);
            angularRoughPID.angularPIDUpdate(followPosition.w);
            if(radius>1 && current.getDistance(pathPositions.get(pathPositions.size()-1)) < radius) {
                radius -= decrementRadius;
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
        packet.put("Current",odometry.getCurrentPosition().toString());
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
        }else{
            motorTakeIn_and_Horizontal.setPower(0);
        }
    }

    private void motorControl(double launchSpeedI, double launchPowerII, boolean takeInFlag){
        motorEjectI.setVelocity(launchSpeedI);
        motorEjectII_and_RightVertical.setPower(launchPowerII);
        if(takeInFlag) {
            motorTakeIn_and_Horizontal.setPower(-1);
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
        sleep(300);
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