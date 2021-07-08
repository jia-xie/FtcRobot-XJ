package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LEDtest extends LinearOpMode {

    RevBlinkinLedDriver lights;
    @Override
    public void runOpMode() throws InterruptedException {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        waitForStart();

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

        while (opModeIsActive());
    }
}
