package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "找环")
public class TfodFindingTest extends OpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "ATgiATj/////AAABmRMerr8SOkjHlkK7C7dYREsyziaAdW3bkHelBM9Dis85dmrAm3IorUDfj5eSM1KmU4TBelqnc45mRHjh6SIcbqJq9CXi3Tznux2WXx9ppfDmQR4HHouK5VzkjAWp6ikMCSebOfrlg1jgiMwQYDmsDYJkHlbkoGCMdQdZLpkZ9J609MEDMQhKfR/aVpQdUlU4R+npBAJXymcpRrKJNQLthsVf8rUN6NXWEN/g9zKi64Hlq9uVtcTjcNoppPWii2UimXVA+JKAlaMYJjchM0PxZdYqBhSDpL3xFkceL3kexbg7a63q9KkE371WY4Baf3Rkksxs0cjTFmyENJZh9C/TfrFwbBd07mGDQmuOBCahHfnD";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    String string = "!";

    @Override
    public void init() {
        initVuforia();
        initTfod();
        tfod.activate();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(tfod,0);
        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
 //               telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                for (Recognition recognition : updatedRecognitions) {
                    string = "(" + (recognition.getLeft() + recognition.getRight()) / 2 + ","
                            + recognition.getBottom() + ")";
                    telemetry.addLine(string);
                }
            }
        }
        else {
            telemetry.addLine(string);
        }
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        tfod.shutdown();
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
