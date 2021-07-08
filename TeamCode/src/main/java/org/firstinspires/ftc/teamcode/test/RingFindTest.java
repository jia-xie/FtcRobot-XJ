package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.ringDetection.FindRingPipeline;
import org.firstinspires.ftc.teamcode.util.ringDetection.Ring;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "test", name = "test: RingFindTest")
@Config
public class RingFindTest extends LinearOpMode
{
    FtcDashboard dashboard;
    OpenCvCamera webcam;

    @Override
    public void runOpMode()
    {
        dashboard = FtcDashboard.getInstance();
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

        telemetry.addLine("Not ready for start");
        telemetry.update();
        while (pipeline.getRings() == null);
        telemetry.addLine("Ready for start");
        telemetry.update();

        FtcDashboard.getInstance().startCameraStream(webcam,0);
        waitForStart();

        while (opModeIsActive())
        {
            while (!pipeline.finishFlag);
            for (Ring ring : pipeline.getRings()) telemetry.addLine(ring.getPosition().toString());

            telemetry.update();
            Thread.yield();
        }
    }
}
