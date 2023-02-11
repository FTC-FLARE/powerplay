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
@Autonomous(name="MM_Auto_Red_Left", group="MM")
public class MM_Auto_Red_Left extends MM_OpMode {
    private final MM_Robot robot = new MM_Robot(this);

    MM_EOCVDetection detector = new MM_EOCVDetection();
    OpenCvCamera camera;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        alliance = MM_EOCVDetection.RED;

        telemetry.addData("Status", "Wait for initialization");
        telemetry.update();
        initCamera();
        robot.init();
        robot.drivetrain.initializeGyroAndEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        totalTime.reset();
        int sleeveColor = detector.getMaxColor();
        robot.lift.chomper.release();
        robot.drivetrain.microscopicDriveInches(1.40);
        robot.drivetrain.strafeInches(-8);
        robot.drivetrain.autoScore();
        while (opModeIsActive() && runtime.seconds() < 0.3) {
        }
        robot.drivetrain.diagonalDriveInches(2, 8);
        robot.drivetrain.rotateToAngle(90); //1/25 - -56.75 under this
        robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(5), 22, -57.5, MM_Drivetrain.DRIVE, 70, 8, false, false);
        robot.drivetrain.rotateToMicroscopicAngle(90);
        robot.drivetrain.correctForTape();
        if (!robot.drivetrain.correctForCone()) {
            robot.parkFromStack(sleeveColor, true);
        } else {
            robot.drivetrain.resetEncoders();
            robot.lift.autoStackCollect(5);
            robot.runSlideandDrive(MM_Slide.SlidePosition.LOW, -10.5,4, false);
            robot.drivetrain.rotateToMicroscopicAngle(90);
            robot.drivetrain.microscopicStrafeInches(2.5);
            robot.lift.scoreCone();
            robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(5), 10.2, -1, 2, 0,5,false, false);
            robot.drivetrain.rotateToMicroscopicAngle(90);
            robot.drivetrain.correctForTape();
            if (!robot.drivetrain.correctForCone()) {
                robot.parkFromStack(sleeveColor, true);
            } else {
                robot.lift.autoStackCollect(4);
                robot.runSlideandDrive(MM_Slide.SlidePosition.LOW, -10.2,4, false);
                robot.drivetrain.rotateToMicroscopicAngle(90);
                robot.drivetrain.microscopicStrafeInches(2.5);
                robot.lift.scoreCone();
                robot.drivetrain.resetEncoders();
                robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(5), 9.2, -1, 2, 0,5,false, false);
                robot.drivetrain.rotateToMicroscopicAngle(90);
                robot.drivetrain.correctForTape();
                if (!robot.drivetrain.correctForCone()) { //add another parameter to check for time because being parked is more worth
                    robot.parkFromStack(sleeveColor, true);
                } else {
                    robot.lift.autoStackCollect(3);
                    robot.drivetrain.resetEncoders();
                    if (!robot.timeToScore(totalTime.seconds(), sleeveColor)) {
                        robot.parkFromStack(sleeveColor, true);
                    } else {
                        robot.drivetrain.microscopicStrafeInches(0.9);
                        robot.runSlideandDrive(MM_Slide.SlidePosition.MEDIUM, -34.2, 5, false);
                        if (robot.timedOut() && robot.drivetrain.stuckOnCone()) {
                            robot.drivetrain.strafe(1); //left
                            runtime.reset();
                            while (opModeIsActive() && runtime.seconds() < 2) {
                            }
                        } else {
                            robot.drivetrain.rotateToMicroscopicAngle(90);
                            robot.lift.scoreCone();
                            robot.drivetrain.resetEncoders();
                        }
                        robot.parkFromJunction(sleeveColor, true);
                    }
                }
            }
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
