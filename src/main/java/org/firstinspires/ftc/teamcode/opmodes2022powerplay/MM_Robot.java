package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Robot {
    private final MM_OpMode opMode;
    public MM_Drivetrain drivetrain;
    public MM_Lift lift;

    static final double MIN_DRIVE_SPEED = 0.23;
    static final double MAX_DRIVE_SPEED = 0.6;
    static final double MIN_STRAFE_POWER = 0.283;
    static final double MAX_STRAFE_POWER = 0.6;
    static final double MIN_ROTATE_POWER = 0.24;
    static final double MAX_ROTATE_POWER = 0.6;

    ElapsedTime runtime = new ElapsedTime();
    private boolean timedOut = false;

    public MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void sleevePark(int sleeveColor, boolean secondCone, boolean thirdCone, boolean right) {
        if (thirdCone && sleeveColor == 0) {
            lift.slide.waitToReachPosition(MM_Slide.SlidePosition.COLLECT);
        } else if(secondCone){
            drivetrain.rotateToAngle(0);
            if (!thirdCone) {
                microscopicRunSlideandDrive(MM_Slide.SlidePosition.COLLECT,3.5, 5);
            } else {
                drivetrain.microscopicDriveInches(3.5);
            }

            if (sleeveColor == MM_EOCVDetection.BLUE){
                if (right) {
                    drivetrain.strafeInches(22);
                } else {
                    drivetrain.strafeInches(-22);
                }
            }else if (sleeveColor == MM_EOCVDetection.YELLOW){
                if (right) {

                } else {
                    drivetrain.strafeInches(-46);
                }
            } else {
                if (right) {
                    drivetrain.strafeInches(46);
                }
            }

            if (thirdCone) {
                lift.slide.waitToReachPosition(MM_Slide.SlidePosition.COLLECT);
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
            lift.slide.waitToReachPosition(MM_Slide.SlidePosition.COLLECT);
            opMode.telemetry.update();

        }
    }

    public void parkFromStack(int maxColor) {
            lift.slide.waitToReachPosition(MM_Slide.SlidePosition.PIVOT_AUTO);
        if (maxColor == MM_EOCVDetection.BLUE) {
            drivetrain.microscopicStrafeInches(1);
            runSlideandDrive(MM_Slide.SlidePosition.COLLECT, -20, 4, false);
        } else if (maxColor == MM_EOCVDetection.YELLOW) {
            drivetrain.microscopicStrafeInches(1);
            runSlideandDrive(MM_Slide.SlidePosition.COLLECT, -44, 5, false);
        } else {
            runSlideandDrive(MM_Slide.SlidePosition.COLLECT, -4, 3, false);
        }
    }

    public void runSlideandDrive(MM_Slide.SlidePosition slidePosition, double inches, double timeoutTime, boolean flipTurner) {
        drivetrain.prepareToDrive(inches);
        lift.slide.moveTowardTarget(slidePosition);

        boolean driveDone = false;
        boolean slideDone = false;
        runtime.reset();

        while (opMode.opModeIsActive() && (!driveDone || !slideDone) && runtime.seconds() < timeoutTime) {
            driveDone = drivetrain.reachedPositionDrive();

            if (flipTurner) {
                if (!slideDone) {
                    slideDone = lift.reachedPositionTurner(); //watch out
                   }
            } else {
                slideDone = lift.slide.reachedPosition();
            }

            opMode.telemetry.addData("inches target", inches);
            opMode.telemetry.addData("slide target", slidePosition);
            opMode.telemetry.update();
        }
    }

    public void runSlideandDiagonalDrive(int ticks, double forwardInches, double strafeInches, int move, int kickInPercent, double timeoutTime, boolean flipTurner) {
        drivetrain.prepareToDiagonalDrive(forwardInches, strafeInches, kickInPercent, move);
        lift.slide.moveTowardTarget(ticks);

        boolean driveDone = false;
        boolean slideDone = false;
        runtime.reset();

        while (opMode.opModeIsActive() && (!driveDone || !slideDone) && runtime.seconds() < timeoutTime) {
            driveDone = drivetrain.reachedPositionDiagonalDrive();

            if (flipTurner) {
                if (!slideDone) {
                    slideDone = lift.reachedPositionTurner();//watch out for this
                }
            } else {
                slideDone = lift.slide.reachedPosition();
            }
            opMode.telemetry.addData("forward inches target", forwardInches);
            opMode.telemetry.addData("strafe inches target", strafeInches);
            opMode.telemetry.addData("slide target", ticks);
            opMode.telemetry.update();
        }
        timedOut = (slideDone && driveDone);
    }

    public void microscopicRunSlideandDrive(MM_Slide.SlidePosition slidePosition, double inches, double timeoutTime) {
        microscopicRunSlideandDrive(slidePosition, inches, timeoutTime, false);
    }

    public void microscopicRunSlideandDrive(MM_Slide.SlidePosition slidePosition, double inches, double timeoutTime, boolean flipTurner) {
        drivetrain.prepareToDrive(inches);
        lift.slide.moveTowardTarget(slidePosition);
        boolean driveDone = false;
        boolean slideDone = false;
        runtime.reset();

        if (flipTurner) {
            boolean turnerDone = false;
            while (opMode.opModeIsActive() && (!driveDone || !slideDone || !turnerDone) && runtime.seconds() < timeoutTime) {
                driveDone = drivetrain.reachedPositionMicroscopicDrive();
                slideDone = lift.slide.reachedPosition();
                if (!turnerDone) {
                    turnerDone = opMode.robot.lift.turner.reachedPosition(lift.slide.tooLowToPivot());
                }
                opMode.telemetry.addData("inches target", inches);
                opMode.telemetry.addData("slide target", slidePosition);
                opMode.telemetry.update();
            }
        } else {
            while (opMode.opModeIsActive() && (!driveDone || !slideDone) && runtime.seconds() < timeoutTime) {
                driveDone = drivetrain.reachedPositionMicroscopicDrive();
                slideDone = lift.slide.reachedPosition();
                opMode.telemetry.addData("inches target", inches);
                opMode.telemetry.addData("slide target", slidePosition);
                opMode.telemetry.update();
            }
        }
    }

    public boolean getTimedOut() {


        return timedOut;
    }

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
        lift = new MM_Lift(opMode);

        opMode.pTurnController.setOutputRange(MIN_ROTATE_POWER, MAX_ROTATE_POWER);
        opMode.pMicroscopicTurnController.setOutputRange(0.1225, MAX_ROTATE_POWER);
        opMode.pLeftDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
        opMode.pRightDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
        opMode.pBackDriveController.setOutputRange(MIN_STRAFE_POWER, MAX_STRAFE_POWER);
        opMode.pLeftDiagDriveController.setOutputRange(0.15, MAX_DRIVE_SPEED);
        opMode.pRightDiagDriveController.setOutputRange(0.15, MAX_DRIVE_SPEED);
    }
}
