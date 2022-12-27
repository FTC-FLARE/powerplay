package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

public class MM_Slide {
    private final MM_OpMode opMode;
    private MM_Turner turner;
    private DcMotor slide = null;
    private DigitalChannel topStop;
    private DigitalChannel bottomStop;

    private final static double SLIDE_POWER = 0.6;
    private final static int MANUAL_INCREMENT = 150;

    private int slideTarget = 0;
    private int stackLevel = 1;

    public enum SlidePosition {
        UNUSED(0),
        COLLECT(0),
        STACK(145),
        GROUND(400),
        LOW_RELEASE(1550),
        PIVOT_POSITION(1600),
        LOW(1750),
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
        if (opMode.gamepad2.right_trigger > 0.1 && !isTriggered(topStop)) {
            setSlideTarget(slide.getCurrentPosition() + MANUAL_INCREMENT);
            stackLevel = 1;
        } else if (opMode.gamepad2.left_trigger > 0.1 && !isTriggered(bottomStop)) {
            setSlideTarget(slide.getCurrentPosition() - MANUAL_INCREMENT);
            stackLevel = 1;
        } else {
            checkSelectHeight();
        }

        if (inDangerZone() || getSlideTarget() < 0) {
            setSlideTarget(0);
        }

        slide.setTargetPosition(getSlideTarget());

        opMode.telemetry.addData("Slide", "Current: %d  Target: %d", slide.getCurrentPosition(), getSlideTarget());
        opMode.telemetry.addData("Top Stop", isTriggered(topStop));
        opMode.telemetry.addData("Bottom Stop", isTriggered(bottomStop));
        opMode.telemetry.addData("Stack Level", stackLevel);
        turner.runTurner(tooLowToPivot());
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
        setSlideTarget(slidePosition.ticks);
        slide.setTargetPosition(getSlideTarget());
    }

    private boolean inDangerZone() {
        if (isTriggered(bottomStop) && getSlideTarget() < slide.getCurrentPosition()) {
            reset();
            return true;
        }

        return false;
    }

    public boolean reachedPosition() {
        return !slide.isBusy() || inDangerZone();
    }

    private boolean isTriggered(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
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

    private void init() {
        turner = new MM_Turner(opMode, this);

        slide = opMode.hardwareMap.get(DcMotor.class, "Slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        reset();

        topStop = opMode.hardwareMap.get(DigitalChannel.class, "topStop");
        bottomStop = opMode.hardwareMap.get(DigitalChannel.class, "bottomStop");

        topStop.setMode(DigitalChannel.Mode.INPUT);  // are these 2 lines needed?
        bottomStop.setMode(DigitalChannel.Mode.INPUT);
    }

    private void reset() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(SLIDE_POWER);
    }
}

