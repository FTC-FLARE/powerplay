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

    private boolean initialized = false;

    private ElapsedTime runtime = new ElapsedTime();

    private int firstCone = MM_Robot.NO_CONE;
    private int secondCone = MM_Robot.NO_CONE;
    private int thirdCone = MM_Robot.NO_CONE;
    private int fourthCone = MM_Robot.NO_CONE;

    @Override
    public void runOpMode() {
        while (!isStarted() && !isStopRequested() && !leftJoystickPressed(GAMEPAD1)) {
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

            if (aPressed(GAMEPAD1)) {
                initialized = true;
                initializeOpmode();
                robot.drivetrain.initializeGyroAndEncoders();
            }

            if (!initialized) {
                telemetry.addLine("Right or left bumper to change alliance");
                telemetry.addLine("Press 'x' to change starting position");
                telemetry.addData(alliance == RED ? "Red" : "Blue", startingPosition == LEFT ? "Left" : "Right");
                telemetry.addLine("Press 'a' to confirm");
            } else if (startingPosition == LEFT){
                telemetry.addData(alliance == RED ? "Red" : "Blue", startingPosition == LEFT ? "Left" : "Right");
                telemetry.addLine("Use dpad up and down to adjust cone configuration");
                telemetry.addLine("current cone configuration:");
                telemetry.addData("low :", lowCones);
                telemetry.addData("medium :", mediumCones);
                telemetry.addData("high :", frontHighCones);
                telemetry.addData("safety park :", signalDanger);
                robot.drivetrain.returnSensorReadings();

            }else {
                telemetry.addData(alliance == RED ? "Red" : "Blue", startingPosition == LEFT ? "Left" : "Right");
                telemetry.addLine("**Only press when completely done with initialization**");
                telemetry.addLine("press left stick to finalize initialization and reset encoders");
                telemetry.addLine("*****************************************************");
                signalDanger = true;
                frontHighCones = 3;
                lowCones = 1;

            }
            telemetry.update();
        }

        robot.drivetrain.resetEncoders();
        telemetry.addLine("encoders reset, DO NOT MOVE ROBOT");
        telemetry.update();


//        robot.drivetrain.returnSensorReadings();
        waitForStart();
        //*************************************** DRIVER HIT PLAY **************************************************
//            code that will always be the same

        startAutosTime();
        parkingColor = detector.getMaxColor();
        if (parkingColor == 4) {
            waitSeconds(0.1);
            parkingColor = detector.getMaxColor();
            if (parkingColor == 4) {
                parkingColor = MM_EOCVDetection.RED;
            }
        }
        if (lowCones == 4) {
            firstCone = MM_Robot.LOW;
            secondCone = MM_Robot.LOW;
            thirdCone = MM_Robot.LOW;
            fourthCone = MM_Robot.LOW;
        } else if (startingPosition == MM_OpMode.RIGHT) {
            firstCone = MM_Robot.RIGHT_HIGH;
            secondCone = MM_Robot.RIGHT_HIGH;
            thirdCone = MM_Robot.RIGHT_HIGH;
        } else if (parkingColor == MM_EOCVDetection.YELLOW) {
            firstCone = MM_Robot.LOW;
            secondCone = MM_Robot.MEDIUM;
            thirdCone = MM_Robot.MEDIUM;
            fourthCone = MM_Robot.FRONT_HIGH;
        } else {
            firstCone = MM_Robot.MEDIUM;
            secondCone = MM_Robot.MEDIUM;
            thirdCone = MM_Robot.MEDIUM;
            fourthCone = MM_Robot.MEDIUM;
        }
        robot.lift.chomper.release();

        camera.stopStreaming();
        camera.closeCameraDevice();
        robot.drivetrain.autoScore();
        robot.collectFromStack();
        robot.scoreOnJunction(firstCone);
        robot.collectFromStack();
        robot.scoreOnJunction(secondCone);

        if (fourthCone != MM_Robot.NO_CONE) {
            robot.collectFromStack();
            handleThirdCone();
            robot.scoreOnJunction(thirdCone);
            robot.collectFromStack();
            handleLastCone();
            robot.scoreOnJunction(fourthCone);
            robot.collectFromStack();
        } else {
            robot.collectFromStack();
            handleLastCone();
            robot.scoreOnJunction(thirdCone);
        }
        robot.park();
    }

    private void handleThirdCone() {
        if (thirdCone != MM_Robot.LOW) { //if you are already not doing the shortest score moves
/*            double targetTimeLeft = robot.getScoreTime(thirdCone) + robot.getCollectTime(thirdCone) + robot.getScoreTime(fourthCone) + robot.getParkTime(fourthCone);
            double actualTimeLeft = timeRemaining();
            if (actualTimeLeft < targetTimeLeft) {
                if (actualTimeLeft > robot.getScoreTime(MM_Robot.MEDIUM) + robot.getCollectTime(MM_Robot.MEDIUM) + robot.getScoreTime(MM_Robot.MEDIUM) + robot.getParkTime(MM_Robot.MEDIUM)) {
                    thirdCone = MM_Robot.MEDIUM;
                } else if (actualTimeLeft > robot.getScoreTime(MM_Robot.LOW) + robot.getCollectTime(MM_Robot.LOW) + robot.getScoreTime(fourthCone) + robot.getParkTime(fourthCone)) {
                    thirdCone = MM_Robot.LOW;
                } else if (actualTimeLeft > robot.getScoreTime(MM_Robot.LOW) + robot.getCollectTime(MM_Robot.LOW) + robot.getScoreTime(MM_Robot.MEDIUM) + robot.getParkTime(MM_Robot.MEDIUM)) {
                    thirdCone = MM_Robot.LOW;
                } else if (actualTimeLeft > robot.getScoreTime(fourthCone) + robot.getParkTime(fourthCone)) {
                    thirdCone = fourthCone;
                } else if (actualTimeLeft > robot.getScoreTime(MM_Robot.MEDIUM) + robot.getParkTime(MM_Robot.MEDIUM)) {
                    thirdCone = MM_Robot.MEDIUM;
                } else if (actualTimeLeft > robot.getScoreTime(MM_Robot.LOW) + robot.getParkTime(MM_Robot.LOW)) {
                    thirdCone = MM_Robot.LOW;
                } else {
                    thirdCone = MM_Robot.NO_CONE;
                }
            }*/
        }
    }

    private void handleLastCone() {
        /*int scoreTarget = fourthCone;
        if (startingPosition == MM_OpMode.RIGHT) {
            scoreTarget = thirdCone;
        }
        double parkTime = robot.getParkTime(scoreTarget);
        double junctionScoreTime = robot.getScoreTime(scoreTarget);
        if (scoreTarget == MM_Robot.LOW) {
            parkTime = robot.getLowCollectandParkTime();
        }
        double timeLeft = timeRemaining();

        if (timeLeft < parkTime + junctionScoreTime) {
            if (startingPosition == MM_OpMode.LEFT) {
                if (timeLeft > robot.getParkTime(MM_Robot.MEDIUM) + robot.getScoreTime(MM_Robot.MEDIUM)) {
                    fourthCone = MM_Robot.MEDIUM;
                } else if (timeLeft > robot.getParkTime(MM_Robot.LOW) + robot.getScoreTime(MM_Robot.LOW)) {
                    fourthCone = MM_Robot.LOW;
                } else {
                    fourthCone = MM_Robot.NO_CONE;
                }
            } else {
                thirdCone = MM_Robot.NO_CONE;
            }
        }*/
    }

    public enum MoveTimes {
        SCORE_LOW(1.9),
        SCORE_MEDIUM(3),
        SCORE_FRONT_HIGH(3.6),
        SCORE_RIGHT_HIGH(3.4),
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
        PARK_YELLOW_RIGHT_HIGH(3.4);



        public final double seconds;

        MoveTimes(double seconds) {
            this.seconds = seconds;
        }
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

}
