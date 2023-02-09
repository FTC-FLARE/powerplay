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

    private boolean xIsPressed = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeOpmode();
        while (!isStarted() && !isStopRequested()){
            if(rightBumperPressed(GAMEPAD1) && alliance == RED){
                alliance = BLUE;
            }else if(leftBumperPressed(GAMEPAD1) && alliance == BLUE){
                alliance = RED;
            } else if (aPressed(GAMEPAD1)) {
                if (startingPosition == LEFT) {
                    startingPosition = RIGHT;
                } else {
                    startingPosition = LEFT;
                }
            } else if (xPressed(GAMEPAD1)) {
                xIsPressed = true;
            }

            if (!xIsPressed) {
                telemetry.addLine("press 'x' after your robot is positioned correctly");
                telemetry.addLine();
                telemetry.addLine("************************");
                telemetry.addLine("************************");
                telemetry.addLine("DONT START ROBOT");
                telemetry.addLine("************************");
                telemetry.addLine("************************");
                telemetry.addLine();
                telemetry.addLine("************************");
                telemetry.addLine("************************");
                telemetry.addLine("NEED TO FINISH INITIALIZATION");
                telemetry.addLine("************************");
                telemetry.addLine("************************");
            } else {
                telemetry.addLine("robot is fully initialized!");
                telemetry.addLine();
                telemetry.addLine("Right or left bumper to change alliance");
                telemetry.addLine("Press 'a' to change starting position");
                telemetry.addData(alliance == RED ? "Red" : "Blue", startingPosition == LEFT ? "Left" : "Right");
            }
            telemetry.update();
        }
        //*************************************** DRIVER HIT PLAY **************************************************
        if (!xIsPressed) {
            robot.drivetrain.initializeGyroAndEncoders();
        }
        //robot.startTotalTime();
        //scorePosition = ;

    }

    private void initializeOpmode() {
        telemetry.addData("Status", "Wait for initialization");
        telemetry.update();

        robot.init();

/*
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
*/


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
