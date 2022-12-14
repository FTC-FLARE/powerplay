package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="MM_Auto_Right", group="MM")
public class MM_Auto_Right extends MM_OpMode {
    private MM_Robot robot = new MM_Robot(this);

    MM_EOCVDetection detector = new MM_EOCVDetection();
    OpenCvCamera camera;

    public static int INCHES = 48;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Wait for initialization");
        telemetry.update();

        initCamera();
        robot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.collector.changePosition(MM_Collector.CLOSED);
        sleep(1000);
        robot.slide.waitToReachPosition(MM_Slide.SlidePosition.LOW);
        robot.collector.flipConeSaver();
        int maxColor = detector.getMaxColor();
        telemetry.addData("Max Color", detector.getMaxColorString());
        telemetry.update();

        //red left
        if (maxColor == MM_EOCVDetection.YELLOW) {
            robot.drivetrain.driveInches(3.5);
            robot.drivetrain.rotateToAngle(-90);
            robot.drivetrain.driveInches(21);
            robot.drivetrain.rotateToAngle(0);
            robot.drivetrain.driveInches(35.25);
            robot.drivetrain.rotateToAngle(90);
            robot.drivetrain.driveInches(.75);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 3) {
            }
            robot.slide.waitToReachPosition(MM_Slide.SlidePosition.LOW_RELEASE);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {
            }
            robot.collector.autoRunCollector();
            runtime.reset();
            while (runtime.seconds() < 1) {
            }
            robot.drivetrain.driveInches(-1.5);
        } else if (maxColor == MM_EOCVDetection.RED) {
            robot.drivetrain.driveInches(5);//5
            robot.drivetrain.rotateToAngle(90);
            robot.drivetrain.driveInches(22);//22
            robot.drivetrain.rotateToAngle(0);
            robot.runSlideandDrive(MM_Slide.SlidePosition.MEDIUM, 33, 5);// 34 medium
            robot.drivetrain.rotateToAngle(-90);
            robot.drivetrain.driveInches(2.3);//2.35
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 3) {
            }
            robot.slide.waitToReachPosition(MM_Slide.SlidePosition.MEDIUM_RELEASE);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {
            }
            robot.collector.autoRunCollector();
            runtime.reset();
            while (runtime.seconds() < 1) {
            }
            robot.drivetrain.driveInches(-1.7);
        } else {
            robot.runSlideandDrive(MM_Slide.SlidePosition.MEDIUM, 39, 5);
            robot.drivetrain.rotateToAngle(90);
            robot.drivetrain.driveInches(3);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 3) {
            }
            robot.slide.waitToReachPosition(MM_Slide.SlidePosition.MEDIUM_RELEASE);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {
            }
            robot.collector.autoRunCollector();
            runtime.reset();
            while (runtime.seconds() < 1) {
            }
            robot.drivetrain.driveInches(-2.5);
        }
        robot.drivetrain.rotateToAngle(0);
        robot.slide.waitToReachPosition(MM_Slide.SlidePosition.COLLECT);


/*      red left 1
        robot.runSlideToPosition(LOW);
        robot.driveInches(3);
        robot.drivetrain.rotateDegrees(90);
        robot.driveInches(23);
        robot.drivetrain.rotateDegrees(0);
        robot.driveInches(33);
        robot.drivetrain.rotateDegrees(-90);
        robot.driveInches(2.5);
        runtime.reset();
        while (runtime.seconds() < 1.5) {
        }
        robot.collector.autoRunCollector(robot.collector.OPEN);
        runtime.reset();
        while (runtime.seconds() < 1) {
        }
        robot.driveInches(-2);
        robot.drivetrain.rotateDegrees(0);*/

/*       red left 2
 robot.runSlideandDrive(MEDIUM, 41, 5);

        robot.drivetrain.rotateDegrees(-90);
        robot.driveInches(3);
        runtime.reset();
        while (runtime.seconds() < 1.5) {
        }
        robot.collector.autoRunCollector(robot.collector.OPEN);
        runtime.reset();
        while (runtime.seconds() < 1) {
        }
        robot.driveInches(-3);
        robot.drivetrain.rotateDegrees(0);
        robot.runSlideToPosition(COLLECT);*/



        telemetry.update();

//        robot.runSlideandDrive(HIGH, 24, 7);
    }

    private void initCamera() {
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
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

    }
}
/*       robot.runSlideToPosition(LOW);

        robot.driveInches(42);
        robot.driveInches(-8);


        robot.drivetrain.rotateDegrees(55);
        robot.driveInches(5);

        robot.collector.autoRunCollector(robot.collector.OPEN);
        robot.driveInches(-5);
        robot.drivetrain.rotateDegrees(0);*/

