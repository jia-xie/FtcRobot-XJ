package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous(group = "test", name = "test: voltage")
public class voltage extends OpMode {
    VoltageSensor voltageSensor;
    @Override
    public void init() {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Voltage Sensor");
    }

    @Override
    public void loop() {

        telemetry.addData("Voltage",voltageSensor.getVoltage());
        telemetry.update();
    }
}
