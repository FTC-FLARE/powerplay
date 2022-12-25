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

    private final double SLIDE_POWER = 0.6;

    private int slideTarget = 0;
    private int stackLevel = 0;

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
        if (isTriggered(bottomStop) && !isHeadedUp()) {  // disengage motor
            stop();
        } else if (opMode.gamepad2.right_trigger > 0.1 && !isTriggered(topStop)) {
            // change triggers to increment position instead?
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(SLIDE_POWER);
            slideTarget = slide.getCurrentPosition();
            stackLevel = 0;
        } else if (opMode.gamepad2.left_trigger > 0.1 && !isTriggered(bottomStop)) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(-SLIDE_POWER);
            slideTarget = slide.getCurrentPosition();
            stackLevel = 0;
        } else if (heightSelected()){ // set target based on button pressed
            startMoving();
        }else {  // hold current target
                startMoving();
        }

        opMode.telemetry.addData("Slide", "Current: %d  Target: %d", slide.getCurrentPosition(), slideTarget);
        opMode.telemetry.addData("Top Stop", isTriggered(topStop));
        opMode.telemetry.addData("Stack Level (+1)", stackLevel + 1);
        turner.runTurner(tooLowToPivot());
    }

    private boolean heightSelected() {
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
        } else if (opMode.dpadDownPressed(opMode.GAMEPAD2)) {
            stackLevel -= 1;
            selectedPosition = SlidePosition.STACK;
        } else if (opMode.dpadUpPressed(opMode.GAMEPAD2)) {
            stackLevel += 1;
            selectedPosition = SlidePosition.STACK;
        }

        if (selectedPosition != SlidePosition.UNUSED) {
            setSlideTarget(selectedPosition.ticks);
            return true;
        }

        return false;
    }

    public void waitToReachPosition(SlidePosition slidePosition) {
        setSlideTarget(slidePosition.ticks);
        startMoving();
        while (opMode.opModeIsActive() && !reachedPosition()) {
        }
    }

    public void startMoving() {
        if (slideTarget == SlidePosition.STACK.ticks) {
            stackLevel = Range.clip(stackLevel, 0, 5);
            slideTarget = SlidePosition.STACK.ticks * (stackLevel - 1);
        } else {
            stackLevel = 0;
        }

        slide.setTargetPosition(slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(SLIDE_POWER);
    }

    private void stop() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideTarget = 0;
    }

    public boolean reachedPosition() {
        if (!slide.isBusy()) {
            return true;
        } else if (isTriggered(bottomStop) && !isHeadedUp()) {
            stop(); // this should hold position instead?
            return true;
        }
        return false;
    }

    private boolean isHeadedUp(){
        return slideTarget > slide.getCurrentPosition() || opMode.gamepad2.right_trigger > .1;
    }

    private boolean isTriggered(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    }

    //the slide is too far down to flip the pivot/turner
    public boolean tooLowToPivot() {
        return slide.getCurrentPosition() < SlidePosition.PIVOT_POSITION.ticks;
    }

    public void setSlideTarget(int slideTarget) {
        this.slideTarget = slideTarget;
    }

    private void init() {
        turner = new MM_Turner(opMode, this);

        slide = opMode.hardwareMap.get(DcMotor.class, "Slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topStop = opMode.hardwareMap.get(DigitalChannel.class, "topStop");
        bottomStop = opMode.hardwareMap.get(DigitalChannel.class, "bottomStop");

        topStop.setMode(DigitalChannel.Mode.INPUT);  // are these 2 lines needed?
        bottomStop.setMode(DigitalChannel.Mode.INPUT);
    }
}
