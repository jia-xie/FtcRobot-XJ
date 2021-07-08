package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;
import org.firstinspires.ftc.teamcode.util.robotAssembly.DriveTrainMotionManagement;

@TeleOp(group = "test", name = "test: Absolute Drive")
public class AbsoluteDriveTrain extends OpMode {

    DriveTrainMotionManagement driveTrain = new DriveTrainMotionManagement();
    ThreeWheelOdometryForYuntai odometry;
    DcMotorEx motorEjectII_and_RightVertical, motorLeftVertical, motorTakeIn_and_Horizontal;

    Position towerShotPosition = new Position(35, 140.5);

    double launchAngle = 10;

    @Override
    public void init() {
        driveTrain.initDriveTrain(hardwareMap,2500);
        motorEjectII_and_RightVertical = hardwareMap.get(DcMotorEx.class, "m21");
        motorLeftVertical = hardwareMap.get(DcMotorEx.class, "m23");
        motorTakeIn_and_Horizontal = hardwareMap.get(DcMotorEx.class,"m22");

        odometry = new ThreeWheelOdometryForYuntai(motorLeftVertical, motorEjectII_and_RightVertical, motorTakeIn_and_Horizontal,
                183,0, 0);
        new Thread(odometry).start();
    }

    @Override
    public void loop() {
        if(!gamepad1.atRest()){
            driveTrain.moveAbsolutely(gamepad1,gamepad1.right_stick_x, odometry);
        }else{
            driveTrain.stop();
        }
    }
    private double getLaunchAngle(ThreeWheelOdometryForYuntai odometry){
        Position relativePosition = odometry.getCurrentPosition().getRelativePosition(towerShotPosition);
        return relativePosition.getAbsoluteAngle() + launchAngle;
    }
}
