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
        while (!isStarted() && !isStopRequested()) {
            updateController();
            if (rightBumperPressed(GAMEPAD1)) {
                alliance = BLUE;
            } else if (leftBumperPressed(GAMEPAD1)) {
                alliance = RED;
            } else if (aPressed(GAMEPAD1)) {
                if (startingPosition == LEFT) {
                    startingPosition = RIGHT;
                } else {
                    startingPosition = LEFT;
                }
            }

            telemetry.addLine("Right or left bumper to change alliance");
            telemetry.addLine("Press 'a' to change starting position");
            telemetry.addData(alliance == RED ? "Red" : "Blue", startingPosition == LEFT ? "Left" : "Right");

            telemetry.update();
        }
        //*************************************** DRIVER HIT PLAY **************************************************
        //robot.startTotalTime();
        //scorePosition = ;

        totalTime.reset();
        if (startingPosition == LEFT) {
            int sleeveColor = detector.getMaxColor();
            robot.lift.chomper.release();
            robot.drivetrain.microscopicDriveInches(1.40);
            robot.drivetrain.strafeInches(-8);
            robot.drivetrain.autoScore();
            while (opModeIsActive() && runtime.seconds() < 0.3) {
            }
            robot.drivetrain.diagonalDriveInches(2, 8);
            robot.drivetrain.rotateToAngle(90); //1/25 - -56.75 under this
            robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(5), 22, -57.5, MM_Drivetrain.DRIVE, 70, 8, false, true);
            robot.drivetrain.rotateToMicroscopicAngle(90);
            robot.drivetrain.correctForTape();
            if (!robot.drivetrain.correctForCone()) {
            } else {
                robot.drivetrain.resetEncoders();
                robot.lift.autoStackCollect(5);
                robot.runSlideandDrive(MM_Slide.SlidePosition.LOW, -10.5,4, false, false);
                robot.drivetrain.rotateToMicroscopicAngle(90);
                robot.drivetrain.microscopicStrafeInches(2.5);
                robot.lift.scoreCone();
                robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(5), 10.2, -1, 2, 0,5,false, false);
                robot.drivetrain.rotateToMicroscopicAngle(90);
                robot.drivetrain.correctForTape();
                if (!robot.drivetrain.correctForCone()) {

                } else {
                    robot.lift.autoStackCollect(4);
                    robot.runSlideandDrive(MM_Slide.SlidePosition.LOW, -10.2,4, false, false);
                    robot.drivetrain.rotateToMicroscopicAngle(90);
                    robot.drivetrain.microscopicStrafeInches(2.5);
                    robot.lift.scoreCone();
                    robot.drivetrain.resetEncoders();
                    robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(5), 9.2, -1, 2, 0,5,false, false);
                    robot.drivetrain.rotateToMicroscopicAngle(90);
                    robot.drivetrain.correctForTape();
                    if (!robot.drivetrain.correctForCone()) { //add another parameter to check for time because being parked is more worth

                    } else {
                        robot.lift.autoStackCollect(3);
                        robot.drivetrain.resetEncoders();
                        if (!robot.timeToScore(totalTime.seconds(), sleeveColor)) {

                        } else {
                            robot.drivetrain.microscopicStrafeInches(0.9);
                            robot.runSlideandDrive(MM_Slide.SlidePosition.MEDIUM, -34.2, 5, false, false);
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
                            robot.park();
                        }
                    }
                }
            }

        }else if (startingPosition == RIGHT){
            robot.drivetrain.getScorerOutOfTheWay();
            robot.lift.slide.waitToReachPosition(robot.lift.slide.stackTicks(1));
            robot.lift.chomper.release();
            robot.drivetrain.microscopicDriveInches(2);
            robot.lift.slide.waitToReachPosition(MM_Slide.SlidePosition.COLLECT);
            robot.lift.chomper.choke();
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 0.3) {
            }
            robot.lift.slide.waitToReachPosition(MM_Slide.SlidePosition.DETECT);
            int sleeveColor = detector.getMaxColor();
            //check to see if you can see color during init with a cone
            robot.runSlideandDiagonalDrive(MM_Slide.SlidePosition.LOW.ticks, 15, 2, MM_Drivetrain.STRAFE, 70, 6, true, false); //test
            robot.lift.scoreCone();
            robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(5), 41, -1.5, 2, 0,8, false, false );
            robot.drivetrain.driveInches(-4);
            robot.drivetrain.rotateToAngle(-90);
            robot.drivetrain.driveInches(23);
            robot.drivetrain.rotateToMicroscopicAngle(-90);
            robot.drivetrain.resetEncoders();
            robot.drivetrain.correctForTape();
            if (!robot.drivetrain.correctForCone()) {

            } else {
                robot.lift.autoStackCollect(5);
                robot.drivetrain.resetEncoders();
                robot.runSlideandDiagonalDrive(MM_Slide.SlidePosition.LOW.ticks, -21, -12, MM_Drivetrain.STRAFE, 90, 7, false, false);
                robot.lift.scoreCone();
                robot.drivetrain.driveInches(-8);
                robot.runSlideandDiagonalDrive(robot.lift.slide.stackTicks(5), 23, 14, 2, 0, 7, false, false);
                robot.drivetrain.correctForTape();
                if (!robot.drivetrain.correctForCone()) {
                } else {
                    robot.lift.autoStackCollect(4);
                    robot.lift.turner.changePosition(MM_Turner.BACK);
                    robot.drivetrain.resetEncoders();
                    robot.runSlideandDiagonalDrive(MM_Slide.SlidePosition.MEDIUM.ticks, -25, -12.2, MM_Drivetrain.STRAFE, 90, 7, false, false);
                    robot.lift.scoreCone();
                    robot.drivetrain.strafeInches(13);
                    robot.park();
                }
            }

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
