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

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Wait for initialization");
        telemetry.update();
        initCamera();
        robot.init();

        int color = detector.getMaxColor();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        robot.lift.chomper.release();
        robot.drivetrain.microscopicDriveInches(1.40);
        robot.drivetrain.strafeInches(-8);
        robot.drivetrain.scoreAndUnscore();
        sleep(1000);
        robot.drivetrain.diagonalDriveInches(2, 8);
        robot.drivetrain.rotateToAngle(90); //1/25 - -56.75 under this
        robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(5), 24.5, -57.9, MM_Drivetrain.DRIVE, 70, 8, false);
        robot.drivetrain.rotateToMicroscopicAngle(90);
        robot.drivetrain.correctForTape();
        robot.drivetrain.correctForCone();
        robot.drivetrain.resetEncoders();
        robot.lift.slide.waitToReachPosition(robot.lift.slide.lowerStackTicks(5));
        robot.lift.chomper.choke();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.3) {
        }
        robot.lift.slide.waitToReachPosition(MM_Slide.SlidePosition.PIVOT_AUTO);
        robot.lift.turner.changePosition(MM_Turner.SIDE);
        robot.runSlideandDrive(MM_Slide.SlidePosition.LOW, -10.5,4, false);
        robot.drivetrain.rotateToMicroscopicAngle(90);
        robot.drivetrain.microscopicStrafeInches(2.5);
        robot.lift.chomper.release();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5) {
        }
        robot.lift.turner.changePosition(MM_Turner.FRONT);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2) {
        }
        robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(4), 10.2, -1, 2, 0,5,false);
        robot.drivetrain.rotateToMicroscopicAngle(90);
        robot.drivetrain.correctForTape();
        robot.drivetrain.correctForCone();
        robot.lift.slide.waitToReachPosition(robot.lift.slide.lowerStackTicks(4));
        robot.lift.chomper.choke();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.3) {
        }
        robot.lift.slide.waitToReachPosition(MM_Slide.SlidePosition.PIVOT_AUTO);
        robot.lift.turner.changePosition(MM_Turner.SIDE);
        robot.runSlideandDrive(MM_Slide.SlidePosition.LOW, -10.2,4, false);
        robot.drivetrain.rotateToMicroscopicAngle(90);
        robot.drivetrain.microscopicStrafeInches(1.9);
        robot.lift.chomper.release();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5) {
        }
        robot.lift.turner.changePosition(MM_Turner.FRONT);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2) {
        }
        robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(3), 9.2, -2.1, 2, 0,5,false);
        robot.drivetrain.rotateToMicroscopicAngle(90);
        robot.drivetrain.correctForTape();
        robot.drivetrain.correctForCone();
        robot.lift.slide.waitToReachPosition(robot.lift.slide.lowerStackTicks(3));
        robot.lift.chomper.choke();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.3) {
        }
        robot.lift.slide.waitToReachPosition(MM_Slide.SlidePosition.PIVOT_AUTO);
        robot.drivetrain.resetEncoders();
        robot.drivetrain.microscopicStrafeInches(0.9);
        robot.lift.turner.changePosition(MM_Turner.SIDE);
        robot.runSlideandDrive(MM_Slide.SlidePosition.MEDIUM, -34.2, 5, false);
        robot.drivetrain.rotateToMicroscopicAngle(90);
        robot.drivetrain.microscopicStrafeInches(1.3);
        robot.lift.chomper.release();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.6) {
        }
        robot.lift.turner.changePosition(MM_Turner.FRONT);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2) {
        }
        robot.drivetrain.resetEncoders();
        if (color == MM_EOCVDetection.RED) {
            robot.runSlideandDiagonalDrive(MM_Slide.SlidePosition.COLLECT.ticks, 35, -1, 2, 0,5,false);
        } else if (color == MM_EOCVDetection.BLUE) {
            robot.runSlideandDiagonalDrive(MM_Slide.SlidePosition.COLLECT.ticks, 9, -1, 2, 0,5,false);
        } else {
            robot.runSlideandDiagonalDrive(MM_Slide.SlidePosition.COLLECT.ticks, -16, -1, 2, 0,5,false);
        }



        //start collection code
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
