package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.MathFunction;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;
import org.firstinspires.ftc.teamcode.util.robotAssembly.DriveTrainMotionManagement;

import static java.lang.Math.max;

@TeleOp(group = "OpMode", name = "云台")
@Config
public class YuntaiOpMode extends OpMode {

    ElapsedTime updateTime = new ElapsedTime();
    DriveTrainMotionManagement driveTrain = new DriveTrainMotionManagement();
    DcMotorEx motorLaunchR, motorLaunchL, motorWheel, motorTimingBelt;
    Servo servoPush, servoOrientationR, servoOrientationL, servoUpDown;
    ColorSensor colorSensor;
    double x, y, w;

    public static double forwardIndex = 0.4;
    public static double backIndex = 0.6;
    public static double[] upIndex = new double[]{0,0,0.02,0.05};
    public static double downIndex = 1;

    public static double launchTowelGoalIndex = 2100;

    public double launchAngle = 90;

    ThreeWheelOdometryForYuntai odometry;
    Position towerShotPosition = new Position(4 * 23 + 11.5, 140.5);

    @Override
    public void init() {
        driveTrain.initDriveTrain(hardwareMap, 0.8);

        motorLaunchL = hardwareMap.get(DcMotorEx.class, "LaunchL");
        motorLaunchR = hardwareMap.get(DcMotorEx.class, "LaunchR");
        motorWheel = hardwareMap.get(DcMotorEx.class, "Wheel");
        motorTimingBelt = hardwareMap.get(DcMotorEx.class, "Timing");

        servoOrientationR = hardwareMap.get(Servo.class, "OR");
        servoOrientationL = hardwareMap.get(Servo.class, "OL");

        servoPush = hardwareMap.get(Servo.class, "P");
        servoUpDown = hardwareMap.get(Servo.class, "U");

        colorSensor = hardwareMap.get(ColorSensor.class, "Color");

        motorTimingBelt.setDirection(DcMotor.Direction.REVERSE);
        motorWheel.setDirection(DcMotor.Direction.REVERSE);
        motorLaunchL.setDirection(DcMotor.Direction.REVERSE);

        motorLaunchL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunchR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odometry = new ThreeWheelOdometryForYuntai(hardwareMap,
                "LF", "RB", "LB",164,
                4 * 23 + 11.5, 3 * 23,0,
                ThreeWheelOdometryForYuntai.UpdateType.LINEAR);
        odometry.resetPulseRecord();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        updateGamePadData();
        driveTrainMove();
        updateLaunchAngle();
        servoMove();
        launchPart();
        takeInPart();

        odometry.updatePositionByStraightPath();
        telemetry.addData("X",odometry.getRobotX());
        telemetry.addData("Y",odometry.getRobotY());
        telemetry.addData("Right", odometry.getYRightPulse());
        telemetry.addData("Left", odometry.getYLeftPulse());
        telemetry.addData("Horizontal", odometry.getXPulse());
        telemetry.addData("Angle",odometry.angle());
    }



    public void updateGamePadData(){
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        w = gamepad1.right_stick_x;
    }

    public void driveTrainMove(){
        driveTrain.movePower(x, y, w);
    }

    public void servoMove(){
        if(gamepad1.dpad_left){
            launchAngle -= 0.2;
        }else if(gamepad1.dpad_right){
            launchAngle  += 0.2;
        }
        launchAngle += gamepad2.left_stick_x;

        launchAngle = Range.clip(launchAngle, 0, 180);
        servoOrientationR.setPosition(launchAngle/180.0);
        servoOrientationL.setPosition(launchAngle/180.0);

        if (gamepad1.x || gamepad2.x){
            servoPush.setPosition(forwardIndex);
        }else{
            servoPush.setPosition(backIndex);
        }


        if(gamepad2.dpad_up){
            servoUpDown.setPosition(upIndex[0]);
        }else if(gamepad2.dpad_down){
            servoUpDown.setPosition(downIndex);
        }

    }



    public void launchPart(){
        if(gamepad1.right_stick_button || gamepad2.right_stick_button){
            motorLaunchL.setVelocity(launchTowelGoalIndex);
            motorLaunchR.setVelocity(launchTowelGoalIndex);
        }
        if(gamepad1.left_stick_button || gamepad2.left_stick_button){
            motorLaunchL.setPower(0);
            motorLaunchR.setPower(0);
        }
    }

    public void updateLaunchAngle(){
        if(gamepad2.right_trigger > 0.5){
            if(updateTime.milliseconds() > 150){



                double absoluteAngle = odometry.getCurrentPosition().getRelativePosition(towerShotPosition).getAbsoluteAngle() - 90;
                launchAngle = odometry.angle() + 90 - absoluteAngle;
                updateTime.reset();
            }
        }
    }

    public void takeInPart(){
        if(gamepad1.right_bumper){

            servoUpDown.setPosition(downIndex);
            motorTimingBelt.setPower(0.9);
            motorWheel.setPower(0.9);

        }else if(gamepad1.left_bumper){
            motorTimingBelt.setPower(-1);
            motorWheel.setPower(-1);
        }else {
            motorTimingBelt.setPower(0.3);
            motorWheel.setPower(0.3);
        }
    }

//    static class ServoT implements Runnable{
//        private final Servo servo;
//        private final Queue<Double> index = new LinkedList<>();
//        private final Queue<Long> waitTime = new LinkedList<>();
//        private volatile boolean isRunnable;
//
//        public ServoT(HardwareMap hardwareMap, String deviceName) {
//            servo = hardwareMap.get(Servo.class, deviceName);
//                    }
//
//        public void setPosition(double index) {
//            this.index.offer(index);
//        }
//
//        public void sleep(long waitTime) {
//            this.waitTime.offer(waitTime);
//        }
//
//        public double getPosition() {
//            return servo.getPosition();
//        }
//
//        public void stop() {
//            isRunnable = false;
//        }
//
//        @Override
//        public void run() {
//            isRunnable = true;
//            while (isRunnable) {
//                if (index.size() > 0)
//                    servo.setPosition(index.poll());
//                if (waitTime.size() > 0) {
//                    try {
//                        Thread.sleep(waitTime.poll());
//                    } catch (InterruptedException e) {
//                        e.printStackTrace();
//                    }
//                }
//                Thread.yield();
//            }
//        }
//    }

}
