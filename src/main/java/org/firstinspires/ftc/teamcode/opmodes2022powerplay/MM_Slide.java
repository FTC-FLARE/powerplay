package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

public class MM_Slide {
    private final MM_OpMode opMode;
    private DcMotor slide = null;
    private DigitalChannel topStop;
    private DigitalChannel bottomStop;
    private ColorSensor slowerizationModule;


    private final static double SLIDE_NORMAL_SPEED = 1;
    private final static double SLIDE_AUTO_SPEED = 1;

    private final int MANUAL_INCREMENT = 150;
    private final static double SLIDE_SLOW_SPEED = 0.3;
    public final static int STACK_LEVEL_INCREMENT = 150;

    private int slideTarget = 0;
    private int stackLevel = 1;
    private double currentSlidePower = SLIDE_NORMAL_SPEED;
    private boolean flipHandled = true;

    private int autoStacklevel = 5;

    public enum SlidePosition {
        FRONT_LOWER(-95),
        RESET(-50),
        UNUSED(0),
        COLLECT(0),
        HOVER_FLOOR(440),//385 emma
        DETECT(700),
        END_POSITION(1700),
        PIVOT_AUTO(1235),
        PIVOT_POSITION(2200),
        LOW(1540),
        MEDIUM(2780),
        HIGH(3915);

        public final int ticks;

        SlidePosition(int ticks) {
            this.ticks = ticks;
        }
    }

    public MM_Slide(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void driverControl() {
        //tape to stop hover
        if (opMode.gamepad2.right_trigger > 0.1 && !atTop()) {
            setSlideTarget(slide.getCurrentPosition() + MANUAL_INCREMENT);
            stackLevel = 1;
        } else if (opMode.gamepad2.left_trigger > 0.1 && !atBottom()) {
            setSlideTarget(slide.getCurrentPosition() - MANUAL_INCREMENT);
            stackLevel = 1;
        } else if (opMode.dpadUpPressed(opMode.GAMEPAD2)){
            setSlideTarget(slide.getCurrentPosition() + 400);
            stackLevel = 1;
        }else if (opMode.dpadDownPressed(opMode.GAMEPAD2)){
            setSlideTarget(slide.getCurrentPosition() -400);
            stackLevel = 1;
        }else {
            checkForSelectedHeight();
        }

        if (opMode.robot.lift.turner.isMoving() && tooLowToPivot() && getSlideTarget() < SlidePosition.PIVOT_POSITION.ticks) {
            setSlideTarget(SlidePosition.PIVOT_POSITION.ticks);
        }

        if (bottomLimit()) {
            opMode.telemetry.addLine("Danger Zone - Resetting Encoder");
            slide.setPower(0);
        }else {
            if (inSlowZone() && headedDown()) {
                currentSlidePower = SLIDE_SLOW_SPEED;
            } else {
                currentSlidePower = SLIDE_NORMAL_SPEED;
            }
            if (opMode.robot.lift.turner.atFront() && flipHandled) {
                if (getSlideTarget() == SlidePosition.MEDIUM.ticks || getSlideTarget() == SlidePosition.HIGH.ticks) {
                    setSlideTarget(getSlideTarget() + SlidePosition.FRONT_LOWER.ticks);
                    flipHandled = false;
                }
            } else if (!flipHandled && opMode.robot.lift.turner.getPosition() == MM_Turner.BACK) {
                if (getSlideTarget() == SlidePosition.MEDIUM.ticks + SlidePosition.FRONT_LOWER.ticks || getSlideTarget() == SlidePosition.HIGH.ticks + SlidePosition.FRONT_LOWER.ticks) {
                    setSlideTarget(getSlideTarget() - SlidePosition.FRONT_LOWER.ticks);
                    flipHandled = true;
                }
            }
            slide.setTargetPosition(getSlideTarget());
            slide.setPower(currentSlidePower);
        }

        opMode.telemetry.addData("Slide", "Current: %d  Target: %d", slide.getCurrentPosition(), getSlideTarget());
        opMode.telemetry.addData("Top Stop", atTop());
        opMode.telemetry.addData("Bottom Stop", atBottom());
        opMode.telemetry.addData("Stack Level", stackLevel);
    }

    public void waitToReachPosition(SlidePosition slidePosition) {
        moveTowardTarget(slidePosition);

        while (opMode.opModeIsActive() && !reachedPosition()) {
            opMode.telemetry.addData("Waiting for Slide", "Current: %d  Target: %d", slide.getCurrentPosition(), getSlideTarget());
            opMode.telemetry.update();
        }
    }

    public boolean waitToReachPosition(int ticks) {
        moveTowardTarget(ticks);

        while (opMode.opModeIsActive() && !reachedPosition()) {
            opMode.telemetry.addData("Waiting for Slide", "Current: %d  Target: %d", slide.getCurrentPosition(), getSlideTarget());
            opMode.telemetry.update();
        }
        return reachedPosition();
    }


    private void checkForSelectedHeight() {
        if (opMode.leftStickYDownPressed(opMode.GAMEPAD2)) {
            changeStack(1);
        } else if (opMode.leftStickYUpPressed(opMode.GAMEPAD2)) {
            changeStack(-1);
        } else {
            int selectedPosition = SlidePosition.UNUSED.ticks;
            if (opMode.leftBumperPressed(opMode.GAMEPAD2)) {
                selectedPosition = SlidePosition.RESET.ticks;
            } else if (opMode.xPressed(opMode.GAMEPAD2)) {
                selectedPosition = SlidePosition.HOVER_FLOOR.ticks;
            } else if (opMode.aPressed(opMode.GAMEPAD2)) {
                selectedPosition = SlidePosition.LOW.ticks;
            } else if (opMode.bPressed(opMode.GAMEPAD2)) {
                selectedPosition = SlidePosition.MEDIUM.ticks;
            } else if (opMode.yPressed(opMode.GAMEPAD2)) {
                selectedPosition = SlidePosition.HIGH.ticks;
            }
            if (selectedPosition != SlidePosition.UNUSED.ticks) {
                stackLevel = 1;
                setSlideTarget(selectedPosition);
            }
        }
    }

    public void changeStack(int change){
        stackLevel = Range.clip(stackLevel + change, 1, 5);
        setSlideTarget(STACK_LEVEL_INCREMENT * (stackLevel - 1) + SlidePosition.HOVER_FLOOR.ticks - 55);
    }

    public int stackTicks(int stackLevel) {
        return STACK_LEVEL_INCREMENT * (stackLevel - 1) + autoScoreLevel(SlidePosition.HOVER_FLOOR);
    }

    public int autoScoreLevel(MM_Slide.SlidePosition slidePosition) {
        if (slidePosition == SlidePosition.LOW) {
            return 1750;
        } else if (slidePosition == SlidePosition.MEDIUM) {
            return 2975;
        } else if (slidePosition == SlidePosition.HIGH) {
            return 4150;
        }
        return 525;
    }

    public int lowerStackTicks(int stackLevel) {
        return STACK_LEVEL_INCREMENT * (stackLevel - 1);
    }

    public void moveTowardTarget(SlidePosition slidePosition) {
        moveTowardTarget(slidePosition.ticks);
    }

    public void moveTowardTarget(int ticks) {
        slide.setPower(SLIDE_AUTO_SPEED);
        setSlideTarget(ticks);
        slide.setTargetPosition(getSlideTarget());
    }

    private boolean bottomLimit() {
        if (atBottom() && headedDown()) {
            reset();
            return true;
        }
        return false;
    }

    private boolean headedDown() {
        return getSlideTarget() < slide.getCurrentPosition();
    }

    private boolean inSlowZone() {
        return slide.getCurrentPosition() < SlidePosition.HOVER_FLOOR.ticks - 125 && slowerizationModule.red() > 1000;
    }

    public boolean reachedPosition() {
        return !slide.isBusy() || (headedDown() && atBottom());
    }

    private boolean atTop() {
        return !topStop. getState();
    }

    private boolean atBottom() {
        return !bottomStop.getState();
    }

    //the slide is too far down to flip the pivot/turner
    public boolean tooLowToPivot() {
        return slide.getCurrentPosition() < SlidePosition.PIVOT_POSITION.ticks;
    }

    public int getSlideTarget() {
        return slideTarget;
    }

    public void setSlideTarget(int slideTarget) {
        this.slideTarget = slideTarget;
    }

    public int getCurrentTicks() {
        return slide.getCurrentPosition();
    }

    private void init() {
        slide = opMode.hardwareMap.get(DcMotor.class, "Slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        reset();

        topStop = opMode.hardwareMap.get(DigitalChannel.class, "topStop");
        bottomStop = opMode.hardwareMap.get(DigitalChannel.class, "bottomStop");
        slowerizationModule = opMode.hardwareMap.get(ColorSensor.class, "slowerizer");

        topStop.setMode(DigitalChannel.Mode.INPUT);  // are these 2 lines needed?
        bottomStop.setMode(DigitalChannel.Mode.INPUT);

    }

    private void reset() {
        setSlideTarget(0);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0);
    }
}

