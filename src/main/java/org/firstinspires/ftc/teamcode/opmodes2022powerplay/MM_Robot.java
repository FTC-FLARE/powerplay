package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.Servo;
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

    static final int START = 5;
    static final int STACK = 4;
    static final int NEARSIDE_HIGH = 3;
    static final int FRONT_HIGH = 2;
    static final int MEDIUM = 1;
    static final int LOW = 0;

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
            double leftDistance = drivetrain.getLeftSonar();
            if (opMode.parkingColor == MM_EOCVDetection.RED) {
                //left sonar start: 51.6 end same
                //front sonar start 5.7 end 29.8
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                    //do nothing
                } else {
                    if (opMode.signalDanger) {
                        drivetrain.microscopicDriveInches(-43.5 + frontDistance);
                    } else {
                        runSlideandDiagonalDrive(MM_Slide.SlidePosition.PIVOT_AUTO.ticks, -54.3 + frontDistance, leftDistance - 51.5 + offset,3, 0,5, false, false);
                    }
                }
            } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                    if (opMode.signalDanger) {
                        drivetrain.microscopicDriveInches(-24.3 + frontDistance);
                    } else {
                        runSlideandDiagonalDrive(MM_Slide.SlidePosition.PIVOT_AUTO.ticks, -30.3 + frontDistance, leftDistance - 51.5 + offset,3, 0,5, false, false);
                    }
                } else {
                    if (opMode.signalDanger) {
                        drivetrain.microscopicDriveInches(-24.3 + frontDistance);
                    } else {
                        runSlideandDiagonalDrive(MM_Slide.SlidePosition.PIVOT_AUTO.ticks, -30.3 + frontDistance, leftDistance - 51.5 + offset,3, 0,5, false, false);
                    }
                }
                drivetrain.engageConePusher();
            } else {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                    if (opMode.signalDanger) {
                        drivetrain.microscopicDriveInches(-43.5 + frontDistance);
                    } else {
                        runSlideandDiagonalDrive(MM_Slide.SlidePosition.PIVOT_AUTO.ticks, -54.3 + frontDistance, leftDistance - 51.5 + offset,3, 0,5, false, false);
                    }
                } else {
                    //do nothing
                }
            }
        } else if (currentPosition == LOW) {
            if (opMode.signalDanger) {
                lift.chomper.release();
                runSlideandDiagonalDrive(lift.slide.stackTicks(5), 10.2, -2, MM_Drivetrain.DRIVE, 40,5,false, true);
                lift.slide.moveTowardTarget(lift.slide.lowerStackTicks(1));

                boolean slideReached = false;
                while (opMode.opModeIsActive() && !slideReached && !drivetrain.isTilted()) {
                    slideReached = lift.slide.reachedPosition();
                    opMode.telemetry.addLine("Waiting for Slide");
                    opMode.telemetry.update();
                }

                if (slideReached) {
                    lift.chomper.choke();
                    opMode.waitSeconds(0.25);
                    lift.slide.waitToReachPosition(lift.slide.lowerStackTicks(2));
                }
                currentPosition = STACK;
                //offset = 24;
                //drivetrain.strafeInches(26);
                park();
            } else {
                drivetrain.engageConePusher();
                drivetrain.strafeInches(-3.25);
                drivetrain.resetEncoders();
                double frontDistance = drivetrain.getFrontSonar();
                double leftDistance = drivetrain.getLeftSonar();

                runtime.reset();
                //front 14.8
                //left 50.5
                if (opMode.parkingColor == MM_EOCVDetection.RED) {
                    if (opMode.startingPosition == MM_OpMode.LEFT) {
                        runSlideandDrive(MM_Slide.SlidePosition.DETECT.ticks, -2.5 + frontDistance, 4, false, false);
                    } else {
                        runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -51.0 + frontDistance, 5, false, false);
                    }
                } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                    if (opMode.startingPosition == MM_OpMode.LEFT) {
                        runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -28.3 + frontDistance, 4,false, false);
                    } else {
                        runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -28.3 + frontDistance, 4,false, false);
                    }
                } else {
                    if (opMode.startingPosition == MM_OpMode.LEFT) {
                        runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -51.0 + frontDistance, 5, false, false);
                    } else {
                        runSlideandDrive(MM_Slide.SlidePosition.DETECT.ticks, -2.5 + frontDistance, 4, false, false);
                    }
                }
            }
        } else if (currentPosition == MEDIUM) {
            drivetrain.engageConePusher();
            drivetrain.resetEncoders();
            double frontDistance = drivetrain.getFrontSonar();
            double leftDistance = drivetrain.getLeftSonar();

            runtime.reset();
            //front 14.8
            //left 50.5
            if (opMode.parkingColor == MM_EOCVDetection.RED) {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                    runSlideandDrive(MM_Slide.SlidePosition.DETECT.ticks, -2.5 + frontDistance, 4, false, false);
                } else {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -51.0 + frontDistance, 5, false, false);
                }
            } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -27.5 + frontDistance, 4,false, false);
                } else {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -27.5 + frontDistance, 4,false, false);
                }
            } else {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -51.0 + frontDistance, 5, false, false);
                } else {
                    runSlideandDrive(MM_Slide.SlidePosition.DETECT.ticks, -2.5 + frontDistance, 4, false, false);
                }
            }
        } else if (currentPosition == FRONT_HIGH) {
            drivetrain.engageConePusher();
            drivetrain.strafeInches(-3.25);
            drivetrain.resetEncoders();
            double frontDistance = drivetrain.getFrontSonar() + 44.5;
            double leftDistance = drivetrain.getLeftSonar();

            runtime.reset();
            //front 14.8
            //left 50.5
            if (opMode.parkingColor == MM_EOCVDetection.RED) {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                    runSlideandDrive(MM_Slide.SlidePosition.DETECT.ticks, -2.5 + frontDistance, 4, false, false);
                } else {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -51.0 + frontDistance, 5, false, false);
                }
            } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -28.3 + frontDistance, 4,false, false);
                } else {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -28.3 + frontDistance, 4,false, false);
                }
            } else {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -51.0 + frontDistance, 5, false, false);
                } else {
                    runSlideandDrive(MM_Slide.SlidePosition.DETECT.ticks, -2.5 + frontDistance, 4, false, false);
                }
            }
        } else if (currentPosition == NEARSIDE_HIGH) {
            drivetrain.engageConePusher();
            drivetrain.resetEncoders();
            double frontDistance = drivetrain.getFrontSonar();
            double leftDistance = drivetrain.getLeftSonar();

            runtime.reset();
            //front 14.8
            //left 50.5
            if (opMode.parkingColor == MM_EOCVDetection.RED) {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                } else {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks, -14, 5, false, false);
                }
            } else if (opMode.parkingColor == MM_EOCVDetection.BLUE) {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                } else {
                    runSlideandDrive(MM_Slide.SlidePosition.END_POSITION.ticks,  10.5, 4,false, false);
                }
            } else {
                if (opMode.startingPosition == MM_OpMode.LEFT) {
                } else {
                    runSlideandDrive(MM_Slide.SlidePosition.DETECT.ticks, 36, 4, false, false);
                }
            }
        } else {
//blind park
        }
        drivetrain.setSignalProtectorPosition(0.5);
        if (opMode.startingPosition == MM_OpMode.LEFT && !(opMode.parkingColor == MM_EOCVDetection.RED)) {
            drivetrain.rotateToAngle(90);
        } else if (opMode.startingPosition == MM_OpMode.RIGHT && !(opMode.parkingColor == MM_EOCVDetection.YELLOW)) {
            drivetrain.rotateToAngle(drivetrain.get90Angle());
        }
        while (opMode.opModeIsActive()) {
            drivetrain.wag();
        }
    }

    public void collectFromStack() {
        lift.turner.autoFrontFlip();
        int signalProtectorPosition = 0;
        if (conesScored == 0) {
            if (opMode.startingPosition == MM_OpMode.LEFT) {
                runSlideandDiagonalDrive(lift.slide.stackTicks(5), 21, -37.5, MM_Drivetrain.DRIVE, 96,6, false, true);
                signalProtectorPosition = 1;
            }
            else {
                runSlideandStrafe(lift.slide.stackTicks(5), -18, 3, false);
                drivetrain.strafeInches(2);
                straightAngle = drivetrain.get180Angle();
                drivetrain.rotateToAngle(straightAngle);
                drivetrain.followTapeToStack();
                //may have to add a drive, not sure
            }
        } else {
            if (lastScored == LOW) {
                runSlideandDiagonalDrive(lift.slide.stackTicks(5 - conesScored), 10.2, -2, MM_Drivetrain.DRIVE, 40,5,false, true);
            } else if (lastScored == MEDIUM) {
                //only works for left and nearside on the right
                runSlideandDiagonalDrive(lift.slide.stackTicks(5 - conesScored), 20, -6, 3, 0,5,false, true);
            } else if (lastScored == FRONT_HIGH) {
                runSlideandDiagonalDrive(lift.slide.stackTicks(5 - conesScored), 44, -4, 3, 0, 6, false, true);
            } else if (lastScored == NEARSIDE_HIGH) {
                runSlideandDiagonalDrive(lift.slide.stackTicks(5 - conesScored), 20, -2, 3, 0,5,false, true);
            }
        }
        double retryCounter = 0;

        if (retryCounter < 2) {
            if (conesScored == 0) {
                drivetrain.setSignalProtectorPosition(signalProtectorPosition);
            }
            double angle = 0;
            if (opMode.startingPosition == MM_OpMode.RIGHT) {
                angle = straightAngle;
            }
            drivetrain.rotateToMicroscopicAngle(angle);
            while (opMode.opModeIsActive() && (!autoStackCollect(5 - conesScored) || !drivetrain.coneCollected())) { //in case robot lifts itself up
                lift.chomper.release();
                drivetrain.resetEncoders();
                drivetrain.driveInches(-12);
                drivetrain.followTapeToStack();
            }
            drivetrain.resetEncoders();
            currentPosition = STACK;
        } else {
            //prob a blind park
        }
    }

    public void scoreOnJunction(int scoreTarget) {
        this.scoreTarget = scoreTarget;
        runtime.reset();
        while (opMode.startingPosition == MM_OpMode.RIGHT && opMode.opModeIsActive() && runtime.seconds() < 0.2) {
            drivetrain.strafe(1);
        }
        lift.turner.autoFlip();
        if (scoreTarget == LOW) {
            runSlideandDrive(MM_Slide.SlidePosition.LOW.ticks, -5, 3, false, true);
        } else if (scoreTarget == MEDIUM) {
            runSlideandDrive(MM_Slide.SlidePosition.MEDIUM.ticks, -30, 4, false, true);
        } else if (scoreTarget == FRONT_HIGH){
            runSlideandDrive(MM_Slide.SlidePosition.HIGH.ticks, -54, 5, false, true);
        } else { //Nearside high
            runSlideandDrive(MM_Slide.SlidePosition.HIGH.ticks, -30, 5, false, true);
        }
        lift.scoreCone();
        currentPosition = scoreTarget;
        lastScored = scoreTarget;
        conesScored+= 1;
        drivetrain.resetEncoders();
    }

    public boolean timeToScore(double totalTime, int maxColor) {
        if (maxColor == MM_EOCVDetection.RED && totalTime > 24.5) {
            return false;
        } else if (maxColor == MM_EOCVDetection.YELLOW && totalTime > 25.5) {
            return false;
        }
        return !(totalTime > 26); //blue
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
        opMode.pFastLeftDriveController.setOutputRange(MIN_DRIVE_SPEED, 1);
        opMode.pFastRightDriveController.setOutputRange(MIN_DRIVE_SPEED, 1);
        opMode.pBackDriveController.setOutputRange(MIN_STRAFE_POWER, MAX_STRAFE_POWER);
        opMode.pLeftDiagDriveController.setOutputRange(0.15, MAX_DRIVE_SPEED);
        opMode.pRightDiagDriveController.setOutputRange(0.15, MAX_DRIVE_SPEED);
    }

    public void setLastScored(int score) {
        lastScored = score;
        conesScored = 1;
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
}
