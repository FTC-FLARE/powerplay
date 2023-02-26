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
                mediumCones = 1;
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
        int sleeveColor = detector.getMaxColor();
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
                robot.park();
            }else {
                robot.scoreOnJunction(MM_Robot.MEDIUM);
                robot.collectFromStack();
                if (sleeveColor == RED){
                    robot.scoreOnJunction(MM_Robot.LOW);
                }else if (sleeveColor == BLUE){
                    robot.scoreOnJunction(MM_Robot.MEDIUM);
                }else{
                    robot.scoreOnJunction(MM_Robot.FRONT_HIGH);
                }
                robot.park();
            }

        }else if (startingPosition == RIGHT){

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
