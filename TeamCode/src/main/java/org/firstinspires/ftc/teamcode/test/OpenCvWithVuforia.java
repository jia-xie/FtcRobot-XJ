package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class OpenCvWithVuforia extends OpMode {

    private static final String VUFORIA_KEY = "ATgiATj/////AAABmRMerr8SOkjHlkK7C7dYREsyziaAdW3bkHelBM9Dis85dmrAm3IorUDfj5eSM1KmU4TBelqnc45mRHjh6SIcbqJq9CXi3Tznux2WXx9ppfDmQR4HHouK5VzkjAWp6ikMCSebOfrlg1jgiMwQYDmsDYJkHlbkoGCMdQdZLpkZ9J609MEDMQhKfR/aVpQdUlU4R+npBAJXymcpRrKJNQLthsVf8rUN6NXWEN/g9zKi64Hlq9uVtcTjcNoppPWii2UimXVA+JKAlaMYJjchM0PxZdYqBhSDpL3xFkceL3kexbg7a63q9KkE371WY4Baf3Rkksxs0cjTFmyENJZh9C/TfrFwbBd07mGDQmuOBCahHfnD";
    private VuforiaLocalizer vuforia;

    @Override
    public void init() {
        initVuforia();

    }

    @Override
    public void loop() {

    }

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
}
