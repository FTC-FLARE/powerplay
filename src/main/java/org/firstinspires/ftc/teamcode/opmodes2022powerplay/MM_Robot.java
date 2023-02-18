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

    static final int HIGH = 2;
    static final int MEDIUM = 1;
    static final int LOW = 0;

    ElapsedTime runtime = new ElapsedTime();
    private boolean timedOut = false;
    private int conesScored = 0;
    private int lastScored = 0;

    public MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void parkFromJunction(int maxColor, boolean left) {
        if (left) {
            if (maxColor == MM_EOCVDetection.RED) {
                runSlideandDiagonalDrive(MM_Slide.SlidePosition.COLLECT.ticks, 35, -1, 2, 0,5,false, false);
            } else if (maxColor == MM_EOCVDetection.BLUE) {
                runSlideandDiagonalDrive(MM_Slide.SlidePosition.COLLECT.ticks, 9, -1, 2, 0,5,false, false);
            } else {
                runSlideandDiagonalDrive(MM_Slide.SlidePosition.COLLECT.ticks, -16, -1, 2, 0,5,false, false);
            }
        } else {
            if (maxColor == MM_EOCVDetection.YELLOW) {
                runSlideandDrive(MM_Slide.SlidePosition.COLLECT, 24, 3, false, false);
            } else if (maxColor == MM_EOCVDetection.BLUE) {
                lift.slide.waitToReachPosition(MM_Slide.SlidePosition.COLLECT.ticks);
            } else {
                runSlideandDrive(MM_Slide.SlidePosition.COLLECT, -24, 3, false, false);
            }
        }
        lift.chomper.choke();
    }

    public void collectFromStack() {
        lift.turner.changePosition(MM_Turner.FRONT);
        if (conesScored == 0) {
            if (opMode.startingPosition == MM_OpMode.LEFT) {
                runSlideandDiagonalDrive(lift.slide.stackTicks(5), 20, -40, MM_Drivetrain.DRIVE, 96,6, false, true);
                drivetrain.followTapeToStack();
                lift.autoStackCollect(5);
                drivetrain.rotateToMicroscopicAngle(0);
            }
            else {
            }
        } else {
            if (lastScored == LOW) {
                runSlideandDiagonalDrive(lift.slide.stackTicks(5), 10.2, -4, MM_Drivetrain.DRIVE, 40,5,false, true);
            } else if (lastScored == MEDIUM) {
                runSlideandDiagonalDrive(lift.slide.stackTicks(5), 28, -9, 3, 0,5,false, true);
            } else {
                runSlideandDiagonalDrive(lift.slide.stackTicks(5), 58.2, -6, 2, 0, 5, false, true);
            }
        }
        drivetrain.followTapeToStack();
        drivetrain.rotateToMicroscopicAngle(90);
        lift.autoStackCollect(5 - conesScored);
        drivetrain.resetEncoders();
/*    if (!drivetrain.correctForCone()) { //TODO NEW METHOD THAT CHECKS DISTANCE FOR CONE FOR FAILSAFE

    } else {

    }*/
    }

    public void scoreOnJunction(int scoreTarget) {
        if (scoreTarget == LOW) {

        } else if (scoreTarget == MEDIUM) {

        } else {

        }
        if (drivetrain.alignedWithJunction()) {
            lift.scoreCone();
            lastScored = scoreTarget;
        }

    }

    public void parkFromStack(int maxColor) {
        drivetrain.resetEncoders();
        lift.slide.waitToReachPosition(MM_Slide.SlidePosition.PIVOT_POSITION);
        if (opMode.startingPosition == MM_OpMode.LEFT) {
            if (maxColor == MM_EOCVDetection.BLUE) {
                drivetrain.microscopicStrafeInches(1);
                drivetrain.driveInches(-20);
            } else if (maxColor == MM_EOCVDetection.YELLOW) {
                drivetrain.microscopicStrafeInches(1);
                drivetrain.driveInches(-48);
                if (drivetrain.stuckOnCone()) {
                    drivetrain.strafe(1);
                    drivetrain.diagonalDriveInches(0, 0, MM_Drivetrain.STRAFE, 50, false);
                }
            } else {
                drivetrain.microscopicStrafeInches(-1);
                drivetrain.driveInches(-5);
            }
        } else {
            if (maxColor == MM_EOCVDetection.BLUE) {
                drivetrain.microscopicStrafeInches(1);
                drivetrain.driveInches(-20);
            } else if (maxColor == MM_EOCVDetection.RED) {
                drivetrain.microscopicStrafeInches(1);
                drivetrain.driveInches(-48);
                if (drivetrain.stuckOnCone()) {
                    drivetrain.strafe(1);
                    drivetrain.diagonalDriveInches(0, 0, MM_Drivetrain.STRAFE, 50, false);
                }
            } else {
                drivetrain.microscopicStrafeInches(-1);
                drivetrain.driveInches(-5);
            }
        }
        lift.chomper.choke();
    }

    public boolean timeToScore(double totalTime, int maxColor) {
        if (maxColor == MM_EOCVDetection.RED && totalTime > 24.5) {
            return false;
        } else if (maxColor == MM_EOCVDetection.YELLOW && totalTime > 25.5) {
            return false;
        }
        return !(totalTime > 26); //blue
    }

    public void runSlideandDrive(MM_Slide.SlidePosition slidePosition, double inches, double timeoutTime, boolean flipTurner, boolean distanceKickOut) {
        drivetrain.prepareToDrive(inches, distanceKickOut);
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

    public void runSlideandStrafe(int slidePosition, double inches, double timeoutTime, boolean flipTurner) {
        drivetrain.prepareToStrafe(inches);
        lift.slide.moveTowardTarget(slidePosition);

        boolean strafeDone = false;
        boolean slideDone = false;
        runtime.reset();

        while (opMode.opModeIsActive() && (!strafeDone || !slideDone) && runtime.seconds() < timeoutTime) {
            strafeDone = drivetrain.reachedPositionStrafe();

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

    public void runSlideandDiagonalDrive(int slideTicks, double forwardInches, double strafeInches, int move, int kickInPercent, double timeoutTime, boolean flipTurner, boolean colorKickOut) {
        drivetrain.prepareToDiagonalDrive(forwardInches, strafeInches, kickInPercent, move, colorKickOut);
        if (colorKickOut) {
            drivetrain.prepareToTapeDrive();
        }
        lift.slide.moveTowardTarget(slideTicks);

        boolean driveDone = false;
        boolean tapeDone = !colorKickOut;
        boolean slideDone = false;
        runtime.reset();

        while (opMode.opModeIsActive() && (!driveDone || !slideDone || !tapeDone) && runtime.seconds() < timeoutTime) {
            if (!tapeDone && driveDone) {
                tapeDone = drivetrain.reachedPositionTapeDrive();
            } else {
                driveDone = drivetrain.reachedPositionDiagonalDrive();
            }

            if (flipTurner) {
                if (!slideDone) {
                    slideDone = lift.reachedPositionTurner();//watch out for this
                }
            } else {
                slideDone = lift.slide.reachedPosition();
            }
            opMode.telemetry.addData("forward inches target", forwardInches);
            opMode.telemetry.addData("strafe inches target", strafeInches);
            opMode.telemetry.addData("slide target", slideTicks);
            opMode.telemetry.addData("robot drive done", driveDone);
            opMode.telemetry.addData("robot slide done", slideDone);
            opMode.telemetry.addData("robot tape done", tapeDone);
            opMode.telemetry.update();
        }
        timedOut = (slideDone && driveDone);
    }

    public boolean timedOut() {
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

    public void setLastScored(int score) {
        lastScored = score;
        conesScored = 1;
    }
}
