package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class MM_Slide {
    private MM_Turner turner;
    private DcMotor slide = null;
    private DigitalChannel topStop;
    private DigitalChannel bottomStop;

    private MM_OpMode opMode;

    private final double SLIDE_POWER = 0.5;

    private int slideCurrent = 0;
    private int slideTarget = 0;
    public int slideLevelTarget = 0;
    private int stackLevel = 0;
    private boolean headedUp = true;
    private boolean isHandled = false;

        //not accurate
    enum slidePosition {
        COLLECT(0),
        STACK(145),
        GROUND(400),
        LOW(1750),
        PIVOT_POSITION(1600),
        MEDIUM(2850),
        HIGH(4000);

        public final int ticks;

        slidePosition(int ticks) {
            this.ticks = ticks;
        }
    }

    public MM_Slide(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void manualRun() {
        if (isTriggered(bottomStop) && opMode.gamepad2.right_trigger <= .1 && !isHandled) {  // disengage motor
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideTarget = 0;
            isHandled = true;
        } else if (opMode.gamepad2.right_trigger > 0.1 && !isTriggered(topStop)) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(SLIDE_POWER);
            slideTarget = slide.getCurrentPosition();
            stackLevel = 0;
            isHandled = false;
        } else if (opMode.gamepad2.left_trigger > 0.1) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(-SLIDE_POWER);
            slideTarget = slide.getCurrentPosition();
            stackLevel = 0;
            isHandled = false;
        } else {  // hold current target
            slide.setTargetPosition(slideTarget);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(SLIDE_POWER);
        }

        slideCurrent = slide.getCurrentPosition();
        opMode.telemetry.addData("Slide", "Current: %d  Target: %d", slideCurrent, slideTarget);
        opMode.telemetry.addData("Top Stop", isTriggered(topStop));
        opMode.telemetry.addData("Stack Level (+1)", stackLevel + 1);
        turner.runTurner();
    }

    public void positionRun() {
        if (opMode.xPressed(opMode.GAMEPAD2)) {
            setSlideTargetAndStart(opMode.COLLECT);
        } else if (opMode.rightJoystickPressed(opMode.GAMEPAD2)) {
            setSlideTargetAndStart(opMode.GROUND);
        } else if (opMode.aPressed(opMode.GAMEPAD2)) {
            setSlideTargetAndStart(opMode.LOW);
        } else if (opMode.bPressed(opMode.GAMEPAD2)) {
            setSlideTargetAndStart(opMode.MEDIUM);
        } else if (opMode.yPressed(opMode.GAMEPAD2)) {
            setSlideTargetAndStart(opMode.HIGH);
        } else if (opMode.dpadDownPressed(opMode.GAMEPAD2)) {
            stackLevel -= 1;
            setSlideTargetAndStart(opMode.STACK);
        } else if (opMode.dpadUpPressed(opMode.GAMEPAD2)) {
            stackLevel += 1;
            setSlideTargetAndStart(opMode.STACK);
        }
    }

    private void setSlideTargetAndStart(int slideLevelTarget) {
        slideTarget = getTicksForLevel(slideLevelTarget);
        this.slideLevelTarget = slideLevelTarget;
        slide.setTargetPosition(slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(SLIDE_POWER);
    }

    public void startMoving(int slideLevelTarget) {
        this.slideLevelTarget = slideLevelTarget;
        slideTarget = getTicksForLevel(slideLevelTarget);
        if (slide.getCurrentPosition() > slideTarget) {
            slide.setPower(-SLIDE_POWER);
            headedUp = false;
        } else if (slide.getCurrentPosition() < slideTarget) {
            slide.setPower(SLIDE_POWER);
            headedUp = true;
        }
    }

    private void stop() {
        slide.setPower(0);
    }

    private int getTicksForLevel(int slideLevelTarget) {
        if (slideLevelTarget == opMode.STACK) {
            if (stackLevel < 1) {
                stackLevel = 0;
                return slideTarget;
            }
            return slidePosition.COLLECT.ticks + (slidePosition.STACK.ticks * (stackLevel - 1));
        } else if (slideLevelTarget == opMode.GROUND) {
            stackLevel = 0;
            return slidePosition.GROUND.ticks;
        } else if (slideLevelTarget == opMode.LOW) {
            stackLevel = 0;
            return slidePosition.LOW.ticks;
        } else if (slideLevelTarget == opMode.MEDIUM) {
            stackLevel = 0;
            return slidePosition.MEDIUM.ticks;
        } else if (slideLevelTarget == opMode.HIGH) {
            stackLevel = 0;
            return slidePosition.HIGH.ticks;
        } else {
            stackLevel = 0;
            return  slidePosition.COLLECT.ticks;
        }
    }

    public int getSlideLevelTarget(int slideLevelTarget) {
        return this.slideLevelTarget;
    }

    public boolean reachedPosition() {
        if (Math.abs(slide.getCurrentPosition() - slideTarget) < 50 || (!headedUp && isTriggered(bottomStop))) { //large for testing purpose
            stop();
            return true;
        }
        return false;
    }

    private boolean isTriggered(DigitalChannel limitSwitch) {  // this can be improved
        if (limitSwitch.getState() == true) {
            return false;
        }
        return true;
    }

    //the slide is too far down to flip the pivot/turner
    public boolean tooLowToPivot() {  // if no additional logic is added, this can be improved
        if (slide.getCurrentPosition() < slidePosition.PIVOT_POSITION.ticks) {  // was 1400
            return true;
        }
        return false;
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

