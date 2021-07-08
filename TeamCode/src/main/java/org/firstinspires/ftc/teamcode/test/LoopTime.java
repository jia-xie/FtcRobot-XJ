package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.odometry.ThreeWheelOdometryForYuntai;

@Autonomous(group = "test", name = "test: multiple threading runtime")
public class LoopTime extends OpMode {
    DcMotorEx motorEjectI, motorEjectII_and_RightVertical, motorTakeIn_and_Horizontal, motorLeftVertical;
    ThreeWheelOdometryForYuntai odometry;
    FtcDashboard dashboard;
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorEjectI = hardwareMap.get(DcMotorEx.class,"m20");
        motorEjectII_and_RightVertical = hardwareMap.get(DcMotorEx.class, "m21");
        motorLeftVertical = hardwareMap.get(DcMotorEx.class, "m23");
        motorTakeIn_and_Horizontal = hardwareMap.get(DcMotorEx.class,"m22");

        odometry = new ThreeWheelOdometryForYuntai(motorLeftVertical,
                motorEjectII_and_RightVertical,
                motorTakeIn_and_Horizontal,
                183,
                110.125, 9,0,
                ThreeWheelOdometryForYuntai.UpdateType.LINEAR);

        new Thread(odometry).start();
        telemetry.addData("Runtime", odometry.getRuntime());
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Runtime", odometry.getRuntime());
        telemetry.update();
    }
}
