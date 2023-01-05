package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Robot {
    private final MM_OpMode opMode;
    public MM_Drivetrain drivetrain;
    public MM_Slide slide;
    public MM_Collector collector;

    static final double MIN_DRIVE_SPEED = 0.24;
    static final double MAX_DRIVE_SPEED = 0.6;
    static final double MIN_STRAFE_POWER = 0.263;
    static final double MAX_STRAFE_POWER = 0.6;
    static final double MIN_ROTATE_POWER = 0.24;
    static final double MAX_ROTATE_POWER = 0.6;

    ElapsedTime runtime = new ElapsedTime();

    public MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void sleevePark(int sleeveColor, boolean secondCone) {
        if(secondCone){
            drivetrain.rotateToAngle(0);
            microscopicRunSlideandDrive(MM_Slide.SlidePosition.COLLECT,5, 5);

            if (sleeveColor == MM_EOCVDetection.BLUE){
                drivetrain.strafeInches(-22);
            }else if (sleeveColor == MM_EOCVDetection.YELLOW){
                drivetrain.strafeInches(-46);
            }

        }else{
            if (sleeveColor == MM_EOCVDetection.RED || sleeveColor == MM_EOCVDetection.YELLOW) {
                drivetrain.driveInches(5); //get away from wall
                double angleTarget = 90;
                if (sleeveColor == MM_EOCVDetection.YELLOW) {
                    angleTarget = -angleTarget;
                    opMode.telemetry.addLine("Traveling to Yellow");
                } else {
                    opMode.telemetry.addLine("Traveling to Red");
                }
                drivetrain.rotateToAngle(angleTarget);
                drivetrain.driveInches(24);
                drivetrain.rotateToAngle(0);
                drivetrain.driveInches(31);
            } else {
                opMode.telemetry.addLine("Traveling to Blue");
                drivetrain.driveInches(42);
                drivetrain.driveInches(-8);
            }
            slide.waitToReachPosition(MM_Slide.SlidePosition.COLLECT);
            opMode.telemetry.update();

        }
    }

    public void runSlideandDrive(MM_Slide.SlidePosition slidePosition, double inches, double timeoutTime) {
        runSlideandDrive(slidePosition, inches, timeoutTime, false);
    }

    public void runSlideandDrive(MM_Slide.SlidePosition slidePosition, double inches, double timeoutTime, boolean flipTurner) {
        drivetrain.prepareToDrive(inches);
        if (flipTurner) {
            slide.moveTowardTarget(slidePosition, true);
        } else {
            slide.moveTowardTarget(slidePosition);
        }

        boolean driveDone = false;
        boolean slideDone = false;
        runtime.reset();

        if (flipTurner) {
            boolean turnerDone = false;
            while (opMode.opModeIsActive() && (!driveDone || !slideDone || !turnerDone) && runtime.seconds() < timeoutTime) {
                driveDone = drivetrain.reachedPositionDrive();
                slideDone = slide.reachedPosition();
                if (!turnerDone) {
                    turnerDone = slide.turner.reachedPosition(slide.tooLowToPivot());
                }
                opMode.telemetry.addData("inches target", inches);
                opMode.telemetry.addData("slide target", slidePosition);
                opMode.telemetry.update();
            }
        } else {
            while (opMode.opModeIsActive() && (!driveDone || !slideDone) && runtime.seconds() < timeoutTime) {
                driveDone = drivetrain.reachedPositionDrive();
                slideDone = slide.reachedPosition();
                opMode.telemetry.addData("inches target", inches);
                opMode.telemetry.addData("slide target", slidePosition);
                opMode.telemetry.update();
            }
        }
    }

    public void microscopicRunSlideandDrive(MM_Slide.SlidePosition slidePosition, double inches, double timeoutTime) {
        microscopicRunSlideandDrive(slidePosition, inches, timeoutTime, false);
    }

    public void microscopicRunSlideandDrive(MM_Slide.SlidePosition slidePosition, double inches, double timeoutTime, boolean flipTurner) {
        drivetrain.prepareToDrive(inches);
        slide.moveTowardTarget(slidePosition);
        boolean driveDone = false;
        boolean slideDone = false;
        runtime.reset();

        if (flipTurner) {
            boolean turnerDone = false;
            while (opMode.opModeIsActive() && (!driveDone || !slideDone || !turnerDone) && runtime.seconds() < timeoutTime) {
                driveDone = drivetrain.reachedPositionMicroscopicDrive();
                slideDone = slide.reachedPosition();
                if (!turnerDone) {
                    turnerDone = slide.turner.reachedPosition(slide.tooLowToPivot());
                }
                opMode.telemetry.addData("inches target", inches);
                opMode.telemetry.addData("slide target", slidePosition);
                opMode.telemetry.update();
            }
        } else {
            while (opMode.opModeIsActive() && (!driveDone || !slideDone) && runtime.seconds() < timeoutTime) {
                driveDone = drivetrain.reachedPositionMicroscopicDrive();
                slideDone = slide.reachedPosition();
                opMode.telemetry.addData("inches target", inches);
                opMode.telemetry.addData("slide target", slidePosition);
                opMode.telemetry.update();
            }
        }
    }

    //    feel free to refactor any names
    public void autoStackCollect(int stackLevel){
        slide.setSlideTarget(MM_Slide.SlidePosition.STACK.ticks * (stackLevel - 1));
        slide.runCollector();
        while (opMode.opModeIsActive() && !slide.reachedPosition()) {
            opMode.telemetry.update();
        }
        collector.autoRunCollector();
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 0.8) {
            opMode.telemetry.update();
        }

    }

    public void autoScore(boolean flipfirst){
        if (flipfirst){
            slide.turner.changeTurnerPosition(slide.turner.BACK);
            runtime.reset();
            while (opMode.opModeIsActive() && runtime.seconds() < 1.5) {
            }
        }
        slide.waitToReachPosition(MM_Slide.SlidePosition.LOW_RELEASE);
        collector.autoRunCollector();
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 1.5){
        }
        slide.waitToReachPosition(MM_Slide.SlidePosition.LOW);

        slide.turner.changeTurnerPosition(slide.turner.FRONT);
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < 1.5){
        }
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
}
