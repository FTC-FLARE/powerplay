package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="MM_Auto", group="MM")
public class MM_Auto extends MM_OpMode {
    private MM_Robot robot = new MM_Robot(this);

    MM_EOCVDetection detector = new MM_EOCVDetection();
    OpenCvCamera camera;

    private boolean xIsPressed = false;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeOpmode();
        robot.drivetrain.initializeGyroAndEncoders();
        while (!isStarted() && !isStopRequested()) {
            updateController();
            if (rightBumperPressed(GAMEPAD1)) {
                alliance = BLUE;
            } else if (leftBumperPressed(GAMEPAD1)) {
                alliance = RED;
            } else if (xPressed(GAMEPAD1)) {
                if (startingPosition == LEFT) {
                    startingPosition = RIGHT;
                } else {
                    startingPosition = LEFT;
                }
            }else if (dpadDownPressed(GAMEPAD1) && autoConeConfiguration < 4){
                autoConeConfiguration = 1;
            }else if (dpadUpPressed(GAMEPAD1) && autoConeConfiguration > 0){
                autoConeConfiguration = 2;
            }

            if (autoConeConfiguration == 1) {
                lowCones = 2;
                mediumCones = 2;
                frontHighCones = 0;
                signalDanger = false;
            }else if (autoConeConfiguration == 2){
                lowCones = 4;
                mediumCones = 0;
                frontHighCones = 0;
                signalDanger = true;
            }

//            robot.drivetrain.returnSensorReadings();
             telemetry.addLine("Right or left bumper to change alliance");
            telemetry.addLine("Press 'x' to change starting position");
            telemetry.addData(alliance == RED ? "Red" : "Blue", startingPosition == LEFT ? "Left" : "Right");
            if (startingPosition == LEFT){
                telemetry.addLine("Use dpad up and down to adjust cone configuration");
                telemetry.addLine("current cone configuration:");
                telemetry.addData("low :", lowCones);
                telemetry.addData("medium :", mediumCones);
                telemetry.addData("high :", frontHighCones);
                telemetry.addData("safety park :", signalDanger);

            }else {
                telemetry.addLine("**Only press when completely done with initialization**");
                telemetry.addLine("press left stick to finalize initialization and reset encoders");
            }
            telemetry.update();
        }
//        robot.drivetrain.returnSensorReadings();
        waitForStart();
        //*************************************** DRIVER HIT PLAY **************************************************
//            code that will always be the same

        totalTime.reset();
        robot.startTimer();
        parkingColor = detector.getMaxColor();
        if (lowCones == 4) {
            lastCone = MM_Robot.LOW;
        } else if (startingPosition == MM_OpMode.RIGHT) {
            lastCone = MM_Robot.RIGHT_HIGH;
        } else if (parkingColor == MM_EOCVDetection.YELLOW) {
            lastCone = MM_Robot.FRONT_HIGH;
        } else {
            lastCone = MM_Robot.MEDIUM;
        }
        robot.lift.chomper.release();
        totalTime.reset();
        camera.stopStreaming();
        camera.closeCameraDevice();
        robot.drivetrain.autoScore();
        robot.collectFromStack();

        if (startingPosition == LEFT) {
            robot.scoreOnJunction(MM_Robot.LOW);
            robot.collectFromStack();
            robot.scoreOnJunction(MM_Robot.LOW);
            robot.collectFromStack();
            if (autoConeConfiguration == 2){
                robot.scoreOnJunction(MM_Robot.LOW);
                robot.collectFromStack();
                robot.scoreOnJunction(MM_Robot.LOW);
            }else {
                robot.scoreOnJunction(MM_Robot.MEDIUM);
                robot.collectFromStack();
                robot.scoreOnJunction(lastCone);

            }
        }else if (startingPosition == RIGHT){
            robot.scoreOnJunction(MM_Robot.RIGHT_HIGH);
            robot.collectFromStack();
            robot.scoreOnJunction(MM_Robot.RIGHT_HIGH);
            robot.collectFromStack();
            robot.scoreOnJunction(MM_Robot.RIGHT_HIGH);
        }
        robot.park();
    }

    private void initializeOpmode() {
        telemetry.addData("Status", "Wait for initialization");
        telemetry.update();

        robot.init();
        initCamera();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
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

    public enum MoveTimes {
        SCORE_LOW(1.9),
        SCORE_MEDIUM(3),
        SCORE_FRONT_HIGH(3.6),
        SCORE_RIGHT_HIGH(3.3),
        CORRECT_TIME(3.7),
        COLLECT_TIME(1.7), //longest possible on 2 cones left
        COLLECT_LOW(3),
        COLLECT_MEDIUM(4.1),
        COLLECT_RIGHT_HIGH(4.2),
        PARK_LEFT_RED_STACK(0),
        PARK_LEFT_BLUE_STACK(1.8),
        PARK_LEFT_YELLOW_STACK(2.2),
        PARK_RED_LOW_COLLECT(3),
        PARK_BLUE_LOW_COLLECT(4.8),
        PARK_YELLOW_LOW_COLLECT(5.1),
        PARK_RED_LOW(1.9),
        PARK_BLUE_LOW(2.4),
        PARK_YELLOW_LOW(2.6),
        PARK_RED_MEDIUM(2.3),
        PARK_BLUE_MEDIUM(2.2),
        PARK_YELLOW_MEDIUM(2.9),
        PARK_YELLOW_HIGH(1.8),
        PARK_RIGHT_RED_STACK(2.2),
        PARK_RIGHT_BLUE_STACK(1.8),
        PARK_RIGHT_YELLOW_STACK(0),
        PARK_RED_RIGHT_HIGH(2.5),
        PARK_BLUE_RIGHT_HIGH(2.2),
        PARK_YELLOW_RIGHT_HIGH(2.1);



        public final double seconds;

        MoveTimes(double seconds) {
            this.seconds = seconds;
        }
    }
}
