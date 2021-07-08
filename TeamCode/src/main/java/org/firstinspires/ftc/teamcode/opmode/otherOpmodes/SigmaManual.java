package org.firstinspires.ftc.teamcode.opmode.otherOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.control.AngularPID;
import org.firstinspires.ftc.teamcode.util.robotAssembly.DriveTrainMotionManagement;
import org.firstinspires.ftc.teamcode.util.Position;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;


@TeleOp(group = "",name = "Sigma OpMode")
public class SigmaManual extends OpMode {   //备用机

    /** no phone operation*/
    FtcDashboard dashboard;

    //Parameters
    double speedIndexR = 0.5;
    int speedMax = 2000;
    double x = 0;
    double y = 0;
    double w = 0;
    double angleOffset = 0;
    double inchPerPulse = 0.02276;
    ElapsedTime runtime = new ElapsedTime();
    //imu:
    BNO055IMU imu;
    Orientation angles;
    //Motors and Servos
    DriveTrainMotionManagement driveTrain = new DriveTrainMotionManagement();
    int ejectVelocity = 1700;

    DcMotorEx motorEjectI = null;

    DcMotorEx motorTakeIn = null;

    Servo servoPush = null;
    Servo servoUpDown = null;

    boolean isEjecting;

    //position
    Position position = new Position(6 * 22.75 - 9,9,0);
    double lastX = 0;
    double lastY = 0;

    DcMotorEx motorLeftVertical = null; //m23
//    DcMotorEx motorRightVertical = null; //m21
    DcMotorEx motorHorizontal = null; //m22

    AngularPID angularPID = new AngularPID(0.04, 0, 0.003, 0, 1, 0.1, 3, 0);
    double targetAngleBlue = 0;
    double targetAngleRed = 0;

    Position targetPositionRed = new Position(4.5 * 22.75, 6 * 22.75, 0);

    @Override
    public void init() {

        imuInit();
        driveTrain.initDriveTrain(hardwareMap,speedMax);

        motorEjectI = hardwareMap.get(DcMotorEx.class,"m20");
        motorTakeIn = hardwareMap.get(DcMotorEx.class,"m22");


        motorHorizontal = hardwareMap.get(DcMotorEx.class,"m22");

        motorLeftVertical  = hardwareMap.get(DcMotorEx.class,"m23");


        servoPush = hardwareMap.get(Servo.class,"sP");
        servoUpDown = hardwareMap.get(Servo.class,"sU");

        angularPID.finish_flag = true;

        dashboard = FtcDashboard.getInstance(); // no phone operation

    }

    @Override
    public void loop() {
        targetAngleRed = -(180 * Math.atan((4.5 * 22.75 - position.x)/(6 * 22.75 - position.y))) / Math.PI;
        targetAngleBlue = (180 * Math.atan((position.x - 1.5 * 22.75)/(6 * 22.75 - position.y)))/ Math.PI;
        angularPID.angularPIDUpdateOnlyForTheSigma(targetAngleRed);
        controllerUpdate();
        if(angularPID.finish_flag) driveTrainMove();
        motorOther();
        servoMove();
        autoPositioning();
        updatePosition();
        if(gamepad1.right_trigger > 0.5){
        eject();}
        if(gamepad1.b){position.x = 6 * 22.75 - 9;position.y = 9;}
        printData();
    }

    public void printData(){
        telemetry.addData("x",position.x);
        telemetry.addData("y",position.y);
        telemetry.addData("targetAngle",targetAngleRed);
        telemetry.addData("ejectVelocity",ejectVelocity);
    }
    public void driveTrainMove(){
        double v = 1;
//        if(gamepad1.y){
//            v = 0.7;
//        } else {
//            v = 1;
//        }
        if(abs(x) > 0.01 || abs(y) > 0.01 || abs(w) > 0.01){
//            if(speedIndexR > 0.2){
//                driveTrain.move(x*0.5,y*0.5,w*0.5);
//            }else{
//                driveTrain.move(x * v,y * v,w * v);
//            }
            driveTrain.move(x * v,y * v,w * v);
        }else{
            driveTrain.stop();
        }
    }

    public void motorOther(){
        if(gamepad1.right_stick_button){
            motorEjectI.setVelocity(ejectVelocity);
        }else if(gamepad1.left_stick_button){
            motorEjectI.setPower(0);
        }else if(gamepad1.left_trigger>0.5){
            motorEjectI.setVelocity(1550); //1600
        }
        //Take In
        if(gamepad1.right_bumper){
            motorTakeIn.setPower(1);
        }else if(gamepad1.left_bumper){
            motorTakeIn.setPower(-1);
        }else{
            motorTakeIn.setPower(0);
        }
    }

    public void servoMove(){
        //Push
        if(gamepad1.x){
            servoPush.setPosition(0.4);
        }else{
            servoPush.setPosition(0.1);
        }

        //UpDown
        if(gamepad1.y){
            servoUpDown.setPosition(1.2);
        }

        //gamepad 1 down
        if(gamepad1.right_bumper){
            servoUpDown.setPosition(.6);
        }

        if(gamepad2.y){
//            servoCatch.setPosition(0.5);//!!!
        }

    }

    public void autoPositioning(){
        if(gamepad1.b){
            angleOffset = -angles.firstAngle;
        }
        if(Math.abs(gamepad1.right_stick_x) > 0.01){
            angularPID.finish_flag = true;
        }
        if(gamepad1.a){
            angularPID.finish_flag = false;
            double dis = position.getDistance(targetPositionRed);
 //           motorEjectI = -() * dis * dis + () * dis + ();
        }
        if(!angularPID.finish_flag){
            double w = angularPID.getAngularPIDOut(angle());
            driveTrain.move(gamepad1.left_stick_x , -gamepad1.left_stick_y, w);
        }
    }



    public void controllerUpdate(){

        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        w = gamepad1.right_stick_x;
//        speedIndexR = gamepad1.right_trigger;
        if(gamepad1Dpad()){
            if(gamepad1.dpad_left){
                w = -0.2;
            }else if(gamepad1.dpad_right){
                w = 0.2;
            }else if(gamepad1.dpad_down){
                y = -0.2;
                ejectVelocity -= 100;
            }else if(gamepad1.dpad_up){
                y = 0.2;
                ejectVelocity += 100;
            }
        }
    }


    public boolean gamepad1Dpad(){
        return gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up;
    }

    private double angle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle + angleOffset; //(-180,180)
    }


    public void imuInit(){
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

    public void multiplePush(int timeInterval){
        runtime.reset();
        //1
        for(runtime.reset();runtime.milliseconds() < timeInterval;){
            servoPush.setPosition(.7);
        }
        for(runtime.reset();runtime.milliseconds() < timeInterval;){
            servoPush.setPosition(.3);
        }
        //2
        for(runtime.reset();runtime.milliseconds() < timeInterval;){
            servoPush.setPosition(.7);
        }
        for(runtime.reset();runtime.milliseconds() < timeInterval;){
            servoPush.setPosition(.3);
        }
        //3
        for(runtime.reset();runtime.milliseconds() < timeInterval;){
            servoPush.setPosition(.7);
        }
        for(runtime.reset();runtime.milliseconds() < timeInterval;){
            servoPush.setPosition(.3);
        }
        //4
        for(runtime.reset();runtime.milliseconds() < timeInterval;){
            servoPush.setPosition(.7);
        }
        for(runtime.reset();runtime.milliseconds() < timeInterval;){
            servoPush.setPosition(.3);
        }
        isEjecting = false;
    }
    public void eject(){
        servoPush.setPosition(.7);
        sleep(100);
        servoPush.setPosition(.3);
        sleep(100);
        servoPush.setPosition(.7);
        sleep(100);
        servoPush.setPosition(.3);
        sleep(100);
        servoPush.setPosition(.7);
        sleep(100);
        servoPush.setPosition(.3);
        sleep(100);
        servoPush.setPosition(.7);
        sleep(100);
        servoPush.setPosition(.3);
        sleep(100);
    }
    public void sleep(int i){
        try {
            Thread.sleep(i);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    public double encoderX(){
        double xLF = driveTrain.motorLF.getCurrentPosition();
        double xLB = driveTrain.motorLB.getCurrentPosition();
        double xRF = driveTrain.motorRF.getCurrentPosition();
        double xRB = driveTrain.motorRB.getCurrentPosition();

        return ((xLF - xLB - xRF + xRB))/4;
    }
    public double encoderY(){
        double yLF = driveTrain.motorLF.getCurrentPosition();
        double yLB = driveTrain.motorLB.getCurrentPosition();
        double yRF = driveTrain.motorRF.getCurrentPosition();
        double yRB = driveTrain.motorRB.getCurrentPosition();

        return (yLF + yLB + yRF + yRB)/4;
    }

    public void updatePosition(){
        double x = encoderX() * inchPerPulse;
        double y = encoderY() * inchPerPulse;

        double deltaX = x - lastX;
        double deltaY = y - lastY;

        double sin = sin(toRadians(angle()));
        double cos = cos(toRadians(angle()));
        position.x += deltaX * cos - deltaY * sin;
        position.y += deltaY * cos + deltaX * sin;

         lastX = x;
        lastY = y;
    }

}
