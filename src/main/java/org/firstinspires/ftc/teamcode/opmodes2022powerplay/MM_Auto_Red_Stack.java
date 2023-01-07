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
@Autonomous(name="MM_Auto_Red_Stack", group="MM")
public class MM_Auto_Red_Stack extends MM_OpMode {
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        robot.collector.changePosition(MM_Collector.CLOSED);
        sleep(500);
        robot.slide.waitToReachPosition(MM_Slide.SlidePosition.DETECT);
        robot.collector.flipConeSaver();
        int maxColor = detector.getMaxColor();
        telemetry.addData("Max Color", detector.getMaxColorString());
        telemetry.update();
        robot.drivetrain.microscopicDriveInches(3);
        robot.drivetrain.strafeInches(23);
        robot.slide.turner.changeTurnerPosition(0);
        robot.runSlideandDrive(MM_Slide.SlidePosition.LOW_HIGH , 41.3, 20);
        robot.drivetrain.microscopicDriveInches(0);
        robot.drivetrain.rotateToAngle(60);
        robot.drivetrain.microscopicDriveInches(-2.5);

        boolean score = true;
        if (!robot.drivetrain.withinJunctionRange()) {
            detector.changeMode(3);
            detector.setConeColor(0);
            score = robot.drivetrain.correctForJunction(detector.getHigherMean());
        }
        detector.changeMode(2);
        if (score) {
            robot.autoScore(false);
            if (detector.goodToCollect()) {
                robot.microscopicRunSlideandDrive(MM_Slide.SlidePosition.CONESAVE_POSITION_FRONT, 6, 5);
                robot.drivetrain.flipDistanceServo();
                robot.autoStackCollect(5);
                robot.microscopicRunSlideandDrive(MM_Slide.SlidePosition.LOW_HIGH, -4.5, 5);
                robot.autoScore(true);
/*                if (maxColor == MM_EOCVDetection.RED) {
                    robot.microscopicRunSlideandDrive(MM_Slide.SlidePosition.CONESAVE_POSITION_FRONT, 5, 5);
                    robot.autoStackCollect(5);
                    robot.microscopicRunSlideandDrive(MM_Slide.SlidePosition.LOW_HIGH, -4.5, 5);
                    robot.autoScore(true);
                }*/

            }
            robot.drivetrain.microscopicDriveInches(3);
            robot.sleevePark(maxColor, true);

        } else {
            robot.slide.turner.changeTurnerPosition(0.885);
            robot.slide.moveTowardTarget(MM_Slide.SlidePosition.COLLECT);
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
