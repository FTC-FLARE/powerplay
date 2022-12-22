package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Robot {
    public MM_Drivetrain drivetrain;
    public MM_Slide slide;
    public MM_Collector collector;

    static final double MIN_DRIVE_SPEED = 0.24;
    static final double MAX_DRIVE_SPEED = 0.6;
    static final double FASTER_MAX_DRIVE_SPEED = 0.0;
    static final double MIN_STRAFE_POWER = 0.0;
    static final double MAX_STRAFE_POWER = 0.0;
    static final double MIN_ROTATE_POWER = 0.24;
    static final double MAX_ROTATE_POWER = 0.6;

    ElapsedTime runtime = new ElapsedTime();

    private MM_OpMode opMode;

    public MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
        slide = new MM_Slide(opMode);
        collector = new MM_Collector(opMode);

        opMode.pTurnController.setOutputRange(MIN_ROTATE_POWER, MAX_ROTATE_POWER);
        opMode.pLeftDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
        opMode.pRightDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
        opMode.pBackDriveController.setOutputRange(MIN_STRAFE_POWER, MAX_STRAFE_POWER);
    }

    public void driveInches(double inches) {
        drivetrain.prepareToDrive(inches);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 5 && !drivetrain.reachedPosition()) {
            opMode.telemetry.addData("inches target", inches);
            opMode.telemetry.update();
        }
    }

    public void sleevePark(int sleeveColor) {
        if (sleeveColor == MM_EOCVSleeveDetection.RED || sleeveColor == MM_EOCVSleeveDetection.YELLOW) {
            driveInches(5); //get away from wall
            double angleTarget = 90;
            if (sleeveColor == MM_EOCVSleeveDetection.YELLOW) {
                angleTarget = -angleTarget;
                opMode.telemetry.addLine("Traveling to Yellow");
            } else {
                opMode.telemetry.addLine("Traveling to Red");
            }
            drivetrain.rotateDegrees(angleTarget);
            driveInches(24);
            drivetrain.rotateDegrees(0);
            driveInches(31);

        } else {
            opMode.telemetry.addLine("Traveling to Blue");
            driveInches(42);
            driveInches(-8);
        }
        runSlideToPosition(opMode.COLLECT);
        opMode.telemetry.update();
    }

    public void runSlideToPosition(int level) {
        slide.startMoving(level);
        while (opMode.opModeIsActive() && !slide.reachedPosition()) {
        }
    }

    public void runSlideandDrive(int level, double inches, double timeoutTime) {
        slide.startMoving(level);
        drivetrain.prepareToDrive(inches);
        boolean driveDone = false;
        boolean slideDone = false;
        runtime.reset();
        while (opMode.opModeIsActive() && (!driveDone || !slideDone) && runtime.seconds() < timeoutTime) {
            driveDone = drivetrain.reachedPosition();
            slideDone = slide.reachedPosition();
            opMode.telemetry.addData("inches target", inches);
            opMode.telemetry.addData("slide target", level);
            opMode.telemetry.update();
        }
    }

}
