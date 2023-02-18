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
@Autonomous(name="MM_Auto_Test", group="MM")
public class MM_Auto_Test extends MM_OpMode {
    private final MM_Robot robot = new MM_Robot(this);

    MM_EOCVDetection detector = new MM_EOCVDetection();
    OpenCvCamera camera;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Wait for initialization");
        telemetry.update();
        initCamera();
        robot.init();
        robot.drivetrain.initializeGyroAndEncoders();
        robot.lift.chomper.release();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.drivetrain.initializeGyroAndEncoders();
        waitForStart();

        alliance = RED;

        totalTime.reset();
        robot.setLastScored(MM_Robot.LOW);
        //start collection code
        robot.lift.slide.waitToReachPosition(MM_Slide.SlidePosition.MEDIUM);
        robot.lift.turner.changePosition(MM_Turner.SIDE);
        sleep(1000);
        robot.lift.turner.changePosition(MM_Turner.FRONT);
        robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(5), 28, -9, 3, 0,5,false, true);
        robot.drivetrain.followTapeToStack();
        robot.lift.autoStackCollect(5);

        while (opModeIsActive()){
            telemetry.addData("Distance", robot.drivetrain.getFrontDistance());
            telemetry.addData("Left Blueness", robot.drivetrain.tempGetLeftBlue());
            telemetry.addData("Right Blueness", robot.drivetrain.tempGetRightBlue());
            telemetry.update();
        }
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
//*/
//    private void firstInitCamera() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
//        // Connect to the camera
//        // Use the SkystoneDetector pipeline
//        // processFrame() will be called to process the frame
//        camera.setPipeline(detector);
//        // Remember to change the camera rotation
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//
//        });
//
//        FtcDashboard.getInstance().startCameraStream(camera, 0);
//
//    }
//}
