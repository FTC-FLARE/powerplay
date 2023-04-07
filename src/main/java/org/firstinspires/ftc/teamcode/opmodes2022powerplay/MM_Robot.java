package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Robot {
    private final MM_OpMode opMode;
    public MM_Drivetrain drivetrain;
    public MM_Lift lift;

    static final double MIN_DRIVE_SPEED = 0.23;
    static final double MAX_DRIVE_SPEED = 0.8;
    static final double MIN_STRAFE_POWER = 0.283;
    static final double MAX_STRAFE_POWER = 0.6;
    static final double MIN_ROTATE_POWER = 0.24;
    static final double MAX_ROTATE_POWER = 0.6;

    static final int START = 5;
    static final int STACK = 4;
    static final int RIGHT_HIGH = 3;
    static final int FRONT_HIGH = 2;
    static final int MEDIUM = 1;
    static final int LOW = 0;
    static final int NO_CONE = -1;

    ElapsedTime runtime = new ElapsedTime();
    private boolean timedOut = false;
    public int conesScored = 0;
    private int lastScored = 0;
    public int currentPosition = START;
    public int scoreTarget = 0;
    private int offset = 0;
    private double straightAngle = 0;

    public MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void park() {
        drivetrain.resetEncoders();
        lift.chomper.choke();
        if (currentPosition == STACK) {
            double frontDistance = drivetrain.getFrontSonar();
            if (opMode.parkingColor == MM_EOCVDetection.RED) {
                if (opMode.startingPosition == MM_OpMode.RIGHT) {
                    drivetrain.microscopicDriveInches(-47 + frontDistance);
                    lift.slide.waitToReachPosition(MM_Slide.SlidePosition.COLLECT);
                }
            } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                drivetrain.microscopicDriveInches(-26.3 + frontDistance);
                lift.slide.waitToReachPosition(MM_Slide.SlidePosition.COLLECT);
            } else {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                    drivetrain.microscopicDriveInches(-49 + frontDistance);
                    lift.slide.waitToReachPosition(MM_Slide.SlidePosition.COLLECT);
                }
            }

        } else if (currentPosition == LOW) {
            double timeCheck = MM_Auto.MoveTimes.PARK_YELLOW_LOW_COLLECT.seconds;
            if (opMode.parkingColor == MM_EOCVDetection.RED) {
                timeCheck = MM_Auto.MoveTimes.PARK_RED_LOW_COLLECT.seconds;
            } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                timeCheck = MM_Auto.MoveTimes.PARK_BLUE_LOW_COLLECT.seconds;
            }
            if (opMode.timeRemaining() > timeCheck) {
                lift.chomper.release();
                runSlideandDiagonalDrive(lift.slide.stackTicks(5), 10.2, -2, MM_Drivetrain.DRIVE, 40,5,false, true);
                currentPosition = STACK;
                lift.slide.moveTowardTarget(lift.slide.lowerStackTicks(1));

                boolean slideReached = false;
                while (opMode.opModeIsActive() && !slideReached && !drivetrain.isTilted() && opMode.timeRemaining() > getParkTime(STACK)) {
                    slideReached = lift.slide.reachedPosition();
                    opMode.telemetry.addLine("Waiting for Slide");
                    opMode.telemetry.update();
                }

                if (slideReached) {
                    lift.chomper.choke();
                    opMode.waitSeconds(0.25);
                    lift.slide.waitToReachPosition(lift.slide.lowerStackTicks(2));
                }
                park();
            } else {
                drivetrain.engageConePusher();
                drivetrain.strafeInches(-3.25);
                drivetrain.resetEncoders();
                double frontDistance = drivetrain.getFrontSonar();
                runtime.reset();
                //front 14.8
                //left 50.5
                if (opMode.parkingColor == MM_EOCVDetection.RED) {
                    runSlideandDrive(lift.slide.stackTicks(5 - conesScored), -2.5 + frontDistance, 4, false, false);
                } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -28.3 + frontDistance, 4,false, false);
                } else {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -51.0 + frontDistance, 5, false, false);
                }
            }
        } else if (currentPosition == MEDIUM) {
            drivetrain.engageConePusher();
            drivetrain.resetEncoders();
            double frontDistance = drivetrain.getFrontSonar();
            runtime.reset();
            if (opMode.parkingColor == MM_EOCVDetection.RED) {
                runSlideandDrive(lift.slide.stackTicks(5 - conesScored), -2.5 + frontDistance, 4, false, false);
            } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -27.5 + frontDistance, 4,false, false);
            } else {
                runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -51.0 + frontDistance, 5, false, false);
            }
        } else if (currentPosition == FRONT_HIGH) {
            drivetrain.engageConePusher();
            drivetrain.strafeInches(-3.25);
            drivetrain.resetEncoders();
            double frontDistance = drivetrain.getFrontSonar();
            if (frontDistance < 25) {
                frontDistance += 44.75;
            }
            runtime.reset();

            if (opMode.parkingColor == MM_EOCVDetection.RED) {
                runSlideandDrive(lift.slide.stackTicks(5 - conesScored), -2.5 + frontDistance, 4, false, false);
            } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -28.3 + frontDistance, 4,false, false);
            } else {
                runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -51.0 + frontDistance, 5, false, false);
            }
        } else if (currentPosition == RIGHT_HIGH) {
            drivetrain.engageConePusher();
            drivetrain.resetEncoders();
            runtime.reset();
            if (opMode.parkingColor == MM_EOCVDetection.RED) {
                runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -13, 5, false, false);
            } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks,  10.5, 4,false, false);
            } else {
                runSlideandDrive(lift.slide.stackTicks(5 - conesScored), 36, 4, false, false);
            }
        } else {
            //blind park
        }

        drivetrain.setSignalProtectorPosition(0.5);
        if (opMode.startingPosition == MM_OpMode.LEFT && !(opMode.parkingColor == MM_EOCVDetection.RED)) {
            drivetrain.rotateToAngle(90);
        } else if (opMode.startingPosition == MM_OpMode.RIGHT && !(opMode.parkingColor == MM_EOCVDetection.YELLOW)) {
            if (currentPosition == STACK) {
                drivetrain.rotateToAngle(drivetrain.get270Angle());
            } else {
                drivetrain.rotateToAngle(drivetrain.get90Angle());
            }
        }
        while (opMode.opModeIsActive()) {
            while (opMode.timeRemaining() > 0.3) {
                drivetrain.wag();
            }
        }
    }

    public double getParkTime(int position) {
        if (opMode.startingPosition == MM_OpMode.LEFT) {
            if (position == STACK) {
                if (opMode.parkingColor == MM_EOCVDetection.RED) {
                    return MM_Auto.MoveTimes.PARK_LEFT_RED_STACK.seconds;
                } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                    return MM_Auto.MoveTimes.PARK_LEFT_BLUE_STACK.seconds;
                } else {
                    return MM_Auto.MoveTimes.PARK_LEFT_YELLOW_STACK.seconds;
                }
            } else if (position == LOW) {
                if (opMode.parkingColor == MM_EOCVDetection.RED) {
                    return MM_Auto.MoveTimes.PARK_RED_LOW.seconds;
                } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                    return MM_Auto.MoveTimes.PARK_BLUE_LOW.seconds;
                } else {
                    return MM_Auto.MoveTimes.PARK_YELLOW_LOW.seconds;
                }
            } else if (position == MEDIUM) {
                if (opMode.parkingColor == MM_EOCVDetection.RED) {
                    return MM_Auto.MoveTimes.PARK_RED_MEDIUM.seconds;
                } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                    return MM_Auto.MoveTimes.PARK_BLUE_MEDIUM.seconds;
                } else {
                    return MM_Auto.MoveTimes.PARK_YELLOW_MEDIUM.seconds;
                }
            } else if (position == FRONT_HIGH) {
                return MM_Auto.MoveTimes.PARK_YELLOW_HIGH.seconds;
            }
        } else {
            if (position == STACK) {
                if (opMode.parkingColor == MM_EOCVDetection.RED) {
                    return MM_Auto.MoveTimes.PARK_RED_RIGHT_HIGH.seconds;
                } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                    return MM_Auto.MoveTimes.PARK_BLUE_RIGHT_HIGH.seconds;
                } else {
                    return MM_Auto.MoveTimes.PARK_YELLOW_RIGHT_HIGH.seconds;
                }
            } else if (position == RIGHT_HIGH) {
                if (opMode.parkingColor == MM_EOCVDetection.RED) {
                    return MM_Auto.MoveTimes.PARK_RIGHT_RED_STACK.seconds;
                } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                    return MM_Auto.MoveTimes.PARK_RIGHT_BLUE_STACK.seconds;
                } else {
                    return MM_Auto.MoveTimes.PARK_RIGHT_YELLOW_STACK.seconds;
                }
            }
        }
        return 5; //blind park guess
    }

    public void collectFromStack() {
        runtime.reset();
        int signalProtectorPosition = 0;
        if (conesScored == 0) {
            if (opMode.startingPosition == MM_OpMode.LEFT) {
                lift.turner.autoFrontFlip();
                runSlideandDiagonalDrive(lift.slide.stackTicks(5), 21, -36.75, MM_Drivetrain.DRIVE, 96,6, false, true);
                signalProtectorPosition = 1;
            }
            else {
                runSlideandStrafe(lift.slide.stackTicks(5), -18, 3, false);
                drivetrain.strafeInches(3);
                straightAngle = drivetrain.get180Angle();
                drivetrain.rotateToAngle(straightAngle);
                drivetrain.followTapeToStack();
                //may have to add a drive, not sure
            }
        } else {
            if (opMode.timeRemaining() > getParkTime(STACK) + getCollectTime(lastScored)) {
                if (lastScored == LOW) {
                    runSlideandDiagonalDrive(lift.slide.stackTicks(5 - conesScored), 10.2, -2, MM_Drivetrain.DRIVE, 40,Math.min(5, opMode.timeRemaining() - getParkTime(STACK)),false, true);
                } else if (lastScored == MEDIUM) {
                    drivetrain.strafe(-1);
                    opMode.waitSeconds(0.22);
                    runSlideandDiagonalDrive(lift.slide.stackTicks(5 - conesScored), 20, -6, 3, 0,Math.min(5, opMode.timeRemaining() - getParkTime(STACK)),false, true);
                } else if (lastScored == FRONT_HIGH) {
                    runSlideandDiagonalDrive(lift.slide.stackTicks(5 - conesScored), 44, -4, 3, 0, Math.min(6, opMode.timeRemaining() - getParkTime(STACK)), false, true);
                } else if (lastScored == RIGHT_HIGH) {
                    runSlideandDiagonalDrive(lift.slide.stackTicks(5 - conesScored), 20, -2, 3, 0,Math.min(5, opMode.timeRemaining() - getParkTime(STACK)),false, true);
                }
                currentPosition = STACK;
            } else {
                park();
            }
        }
        if (opMode.timeRemaining() > MM_Auto.MoveTimes.COLLECT_TIME.seconds + getParkTime(STACK)) {
            if (conesScored == 0) {
                drivetrain.setSignalProtectorPosition(signalProtectorPosition);
            }
            double angle = 0;
            if (opMode.startingPosition == MM_OpMode.RIGHT) {
                angle = straightAngle + 3;
            }
            drivetrain.rotateToMicroscopicAngle(angle);
            while (opMode.timeRemaining() > MM_Auto.MoveTimes.COLLECT_TIME.seconds + getParkTime(STACK) + 0.2 && opMode.opModeIsActive() && (!autoStackCollect(5 - conesScored) || !drivetrain.coneCollected())) {
                lift.chomper.release();
                if (opMode.timeRemaining() < MM_Auto.MoveTimes.COLLECT_TIME.seconds + MM_Auto.MoveTimes.CORRECT_TIME.seconds + getParkTime(STACK) + 0.2) {
                    park();
                } else {
                    drivetrain.resetEncoders();

                    drivetrain.driveInches(-12);
                    drivetrain.followTapeToStack();
                }
            }
            if (opMode.startingPosition == MM_OpMode.RIGHT) {
                drivetrain.rotateToMicroscopicAngle(straightAngle);
            }
            drivetrain.resetEncoders();
        } else {
            park();
        }

    }

    public boolean autoStackCollect(int stackLevel){
        lift.slide.moveTowardTarget(lift.slide.lowerStackTicks(stackLevel));

        boolean slideReached = false;
        while (opMode.opModeIsActive() && !slideReached && !drivetrain.isTilted()) {
            slideReached = lift.slide.reachedPosition();
            opMode.telemetry.addLine("Waiting for Slide");
            opMode.telemetry.update();
        }

        if (slideReached) {
            lift.chomper.choke();
            opMode.waitSeconds(0.25);
            lift.slide.waitToReachPosition(MM_Slide.SlidePosition.PIVOT_AUTO);
            return true;
        }
        lift.slide.waitToReachPosition(lift.slide.stackTicks(5));
        return false;
    }

    public double getCollectTime(int lastScored) {
        if (lastScored == LOW) {
            return MM_Auto.MoveTimes.COLLECT_LOW.seconds;
        } else if (lastScored == MEDIUM) {
            return MM_Auto.MoveTimes.COLLECT_MEDIUM.seconds;
        }
        return MM_Auto.MoveTimes.COLLECT_RIGHT_HIGH.seconds;
    }

    public double getLowCollectandParkTime() {
        if (opMode.parkingColor == MM_EOCVDetection.RED) {
            return MM_Auto.MoveTimes.PARK_RED_LOW_COLLECT.seconds;
        } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
            return MM_Auto.MoveTimes.PARK_BLUE_LOW_COLLECT.seconds;
        }
        return MM_Auto.MoveTimes.PARK_YELLOW_LOW_COLLECT.seconds;
    }

    public void scoreOnJunction(int scoreTarget) {
        if (opMode.timeRemaining() > getScoreTime(scoreTarget) + getParkTime(scoreTarget) && scoreTarget != NO_CONE) {
            this.scoreTarget = scoreTarget;
            runtime.reset();
            lift.turner.autoFlip();
            if (scoreTarget == LOW) {
                runSlideandDrive(lift.slide.autoScoreLevel(MM_Slide.SlidePosition.LOW), -5, Math.min(2.5, opMode.timeRemaining() - getParkTime(LOW)), false, true);
            } else if (scoreTarget == MEDIUM) {
                runSlideandDrive(MM_Slide.SlidePosition.MEDIUM.ticks, -30, Math.min(3.5, opMode.timeRemaining() - getParkTime(MEDIUM)), false, true);
            } else if (scoreTarget == FRONT_HIGH){
                runSlideandDrive(MM_Slide.SlidePosition.HIGH.ticks, -54, Math.min(4.5, opMode.timeRemaining() - getParkTime(FRONT_HIGH)), false, true);
            } else { //Nearside high
                runSlideandDrive(MM_Slide.SlidePosition.HIGH.ticks, -30, Math.min(3.7, opMode.timeRemaining() - getParkTime(RIGHT_HIGH)), false, true);
            }
            currentPosition = scoreTarget;
            lastScored = scoreTarget;
            if (!drivetrain.getAlignedWithJunction()) {
                if (opMode.timeRemaining() > 1.15 + getParkTime(scoreTarget)) { //the while in drive time plus score time
                    drivetrain.driveUntilJunction();
                } else {
                    park();
                }
            }
            lift.scoreCone();
            conesScored+= 1;
            drivetrain.resetEncoders();
        } else {
            park();
        }
    }

    public double getScoreTime(int scoreTarget) {
        if (scoreTarget == LOW) {
            return MM_Auto.MoveTimes.SCORE_LOW.seconds;
        } else if (scoreTarget == MEDIUM) {
            return MM_Auto.MoveTimes.SCORE_MEDIUM.seconds;
        } else if (scoreTarget == FRONT_HIGH) {
            return MM_Auto.MoveTimes.SCORE_FRONT_HIGH.seconds;
        }
        return MM_Auto.MoveTimes.SCORE_RIGHT_HIGH.seconds;
    }

    public void runSlideandDrive(int slidePosition, double inches, double timeoutTime, boolean flipTurner, boolean distanceKickOut) {
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
            } else if (!driveDone) {
                driveDone = drivetrain.reachedPositionDiagonalDrive() || lastScored == MEDIUM;
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

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
        lift = new MM_Lift(opMode);

        opMode.pTurnController.setOutputRange(MIN_ROTATE_POWER, MAX_ROTATE_POWER);
        opMode.pMicroscopicTurnController.setOutputRange(0.1225, MAX_ROTATE_POWER);
        opMode.pLeftDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
        opMode.pRightDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
        opMode.pFastLeftDriveController.setOutputRange(MIN_DRIVE_SPEED, 1);
        opMode.pFastRightDriveController.setOutputRange(MIN_DRIVE_SPEED, 1);
        opMode.pBackDriveController.setOutputRange(MIN_STRAFE_POWER, MAX_STRAFE_POWER);
        opMode.pLeftDiagDriveController.setOutputRange(0.15, MAX_DRIVE_SPEED);
        opMode.pRightDiagDriveController.setOutputRange(0.15, MAX_DRIVE_SPEED);
    }

}
