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

        //not accurate
    enum slidePosition {
        COLLECT(0),
        GROUND(300),
        LOW(1000),
        PIVOT_POSITION(1400),
        MEDIUM(2000),
        HIGH(3500);

        public final int ticks;

        private slidePosition(int ticks) {
            this.ticks = ticks;
        }
    }

    public MM_Slide(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void manualRun() {
        if (isTriggered(bottomStop) && opMode.gamepad2.right_trigger <= .1) {  // disengage motor
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideTarget = 0;
        } else if (opMode.gamepad2.right_trigger > 0.1 && !isTriggered(topStop)) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(SLIDE_POWER);
            slideTarget = slide.getCurrentPosition();
        } else if (opMode.gamepad2.left_trigger > 0.1) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(-SLIDE_POWER);
            slideTarget = slide.getCurrentPosition();
        } else {  // hold current position
            slide.setTargetPosition(slideTarget);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(SLIDE_POWER);
        }

        slideCurrent = slide.getCurrentPosition();
        opMode.telemetry.addData("Slide", "Current: %d  Target: %d", slideCurrent, slideTarget);
        opMode.telemetry.addData("Top Stop", isTriggered(topStop));
        turner.runTurner();
    }

    public void startMoving(int ticks) {
        if (slide.getCurrentPosition() > ticks) {
            slide.setPower(-SLIDE_POWER);
        } else if (slide.getCurrentPosition() < ticks) {
            slide.setPower(SLIDE_POWER);
        }
        slideTarget = ticks;
    }

    public boolean reachedPosition() {
        if (Math.abs(slide.getCurrentPosition() - slideTarget) < 30 || isTriggered(topStop) || isTriggered(bottomStop)) {
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
        if (slide.getCurrentPosition() < 1050) {  // was 1400
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

