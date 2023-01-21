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
    private ColorSensor colorSensor;


    private final static double SLIDE_NORMAL_SPEED = 0.6;
    private final static int MANUAL_INCREMENT = 150;
    private final static int SLOW_INCREMENT = 50;
    private final static double SLIDE_SLOW_SPEED = 0.3;

    private int slideTarget = 0;
    private int stackLevel = 1;
    private double currentSlidePower = SLIDE_NORMAL_SPEED;
    private boolean slowZone = true;

    private int autoStacklevel = 5;


    public enum SlidePosition {
        UNUSED(0),
        COLLECT(0),
        STACK(125),
        GROUND(400),
        DETECT(700),
        LIFT(900),
        LOW_RELEASE(1550),
        PIVOT_POSITION(1600),
        LOW(1750),
        LOW_HIGH(1900),
        MEDIUM_RELEASE(2650),
        MEDIUM(2850),
        HIGH_RELEASE(3900),
        HIGH(4000);

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
        if (opMode.gamepad2.right_trigger > 0.1 && !atTop()) {
            setSlideTarget(slide.getCurrentPosition() + MANUAL_INCREMENT);
            stackLevel = 1;
        } else if (opMode.gamepad2.left_trigger > 0.1 && !atBottom()) {
            setSlideTarget(slide.getCurrentPosition() - MANUAL_INCREMENT);
            stackLevel = 1;
        } else {
            checkSelectHeight();
        }

        if (opMode.robot.lift.turner.isMoving() && tooLowToPivot() && getSlideTarget() < SlidePosition.PIVOT_POSITION.ticks) {
            setSlideTarget(SlidePosition.PIVOT_POSITION.ticks);
        }

        if (bottomLimit()) {
            opMode.telemetry.addLine("Danger Zone - Resetting Encoder");
        }else {
            if (inSlowZone() && headedDown()) {
                currentSlidePower = SLIDE_SLOW_SPEED;
            } else {
                currentSlidePower = SLIDE_NORMAL_SPEED;
            }
            slide.setTargetPosition(getSlideTarget());
            slide.setPower(currentSlidePower);
        }

        opMode.telemetry.addData("Slide", "Current: %d  Target: %d", slide.getCurrentPosition(), getSlideTarget());
        opMode.telemetry.addData("Top Stop", atTop());
        opMode.telemetry.addData("Bottom Stop", atBottom());
        opMode.telemetry.addData("Stack Level", stackLevel);
        opMode.telemetry.addData("red", colorSensor.red());
    }

    public void waitToReachPosition(SlidePosition slidePosition) {
        moveTowardTarget(slidePosition);

        while (opMode.opModeIsActive() && !reachedPosition()) {
            opMode.telemetry.addData("Waiting for Slide", "Current: %d  Target: %d", slide.getCurrentPosition(), getSlideTarget());
            opMode.telemetry.update();
        }
    }

    private void checkSelectHeight() {
        if (opMode.dpadUpPressed(opMode.GAMEPAD2)) {
            changeStack(1);
        } else if (opMode.dpadDownPressed(opMode.GAMEPAD2)) {
            changeStack(-1);
        } else {
            SlidePosition selectedPosition = SlidePosition.UNUSED;
            if (opMode.xPressed(opMode.GAMEPAD2)) {
                selectedPosition = SlidePosition.COLLECT;
            } else if (opMode.rightJoystickPressed(opMode.GAMEPAD2)) {
                selectedPosition = SlidePosition.GROUND;
            } else if (opMode.aPressed(opMode.GAMEPAD2)) {
                selectedPosition = SlidePosition.LOW;
            } else if (opMode.bPressed(opMode.GAMEPAD2)) {
                selectedPosition = SlidePosition.MEDIUM;
            } else if (opMode.yPressed(opMode.GAMEPAD2)) {
                selectedPosition = SlidePosition.HIGH;
            }

            if (selectedPosition != SlidePosition.UNUSED) {
                stackLevel = 1;
                setSlideTarget(selectedPosition.ticks);
            }
        }
    }

    public void changeStack(int change){
        stackLevel = Range.clip(stackLevel + change, 1, 5);
        setSlideTarget(SlidePosition.STACK.ticks * (stackLevel - 1));
    }

    public void moveTowardTarget(SlidePosition slidePosition) {
        moveTowardTarget(slidePosition.ticks);
    }

    public void moveTowardTarget(int ticks) {
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
        return colorSensor.red() > 1000;
    }

    public boolean reachedPosition() {
        return !slide.isBusy() || bottomLimit();
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
        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "colorSensor");

        topStop.setMode(DigitalChannel.Mode.INPUT);  // are these 2 lines needed? //3 lines
        bottomStop.setMode(DigitalChannel.Mode.INPUT);

    }

    private void reset() {
        setSlideTarget(0);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(SLIDE_NORMAL_SPEED);
    }
}

