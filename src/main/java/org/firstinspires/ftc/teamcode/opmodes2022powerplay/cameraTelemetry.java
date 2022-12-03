package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="OpenCV Telemetry", group="MM")
public class cameraTelemetry extends LinearOpMode {

    int width = 320;
    int height = 240;

    MM_EOCVTelemetry detector = new MM_EOCVTelemetry();
    OpenCvCamera camera;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        // Connect to the camera
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        camera.setPipeline(detector);
        // Remember to change the camera rotation
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
             @Override
             public void onOpened() {
                 camera.startStreaming(width, height,  OpenCvCameraRotation.UPRIGHT);
             }

             @Override
             public void onError(int errorCode) {

             }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("columns", detector.columns());
            telemetry.addData("Hue", detector.getMean1());
            telemetry.addData("Sat", detector.getMean2());
            telemetry.addData("Val", detector.getMean3());

            telemetry.addData("Frame Count", camera.getFrameCount());
            telemetry.update();
        }
    }
}
