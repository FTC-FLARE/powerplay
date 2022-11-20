package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MM_Slide {

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

    public MM_Turner turner;
    DigitalChannel topStop;
    DigitalChannel bottomStop;
    private DcMotor slide = null;

    private MM_OpMode opMode;

    final double SLIDE_POWER = 0.35;

    private int slideCurrent = 0;
    private int slideTarget = 0;

    public MM_Slide(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    private void init() {
        slide = opMode.hardwareMap.get(DcMotor.class, "Slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turner = new MM_Turner(opMode, this);

        topStop = opMode.hardwareMap.get(DigitalChannel.class, "topStop");
        bottomStop = opMode.hardwareMap.get(DigitalChannel.class, "bottomStop");

        topStop.setMode(DigitalChannel.Mode.INPUT);
        bottomStop.setMode(DigitalChannel.Mode.INPUT);
    }

    public void manualRun() {
        if (opMode.gamepad2.left_trigger > 0.1 && !isTriggered(bottomStop)) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(-SLIDE_POWER);
        } else if (opMode.gamepad2.right_trigger > 0.1 && !isTriggered(topStop)) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(SLIDE_POWER);
        } else {
            slide.setTargetPosition(slideCurrent);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (isTriggered(bottomStop)) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        slideCurrent = slide.getCurrentPosition();
        opMode.telemetry.addData("Slide Current", slideCurrent);
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

    private boolean isTriggered(DigitalChannel limitSwitch) {
        if (limitSwitch.getState() == true) {
            return false;
        }
        return true;
    }

    //terrible name, but means that the slide is too far down to flip the pivot/turner
    public boolean lowFlip() {
        if (slide.getCurrentPosition() < 1400) {
            return true;
        }
        return false;
    }
}

