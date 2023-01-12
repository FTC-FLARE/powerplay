package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MM_Turner{
    private final MM_OpMode opMode;
    private Servo turner = null;

    final double FRONT = 0.885;
    final double BACK = 0;
    static final double BACK_TURN_INCREMENT = -0.020;
    static final double FRONT_TURN_INCREMENT = 0.025;

    private ElapsedTime timer = new ElapsedTime();
    private boolean turnerDone = false;
    private boolean isMoving = false;
    private double currentPosition = FRONT;
    private double targetPosition = FRONT;
    private double turnIncrement = BACK_TURN_INCREMENT;

    public MM_Turner(MM_OpMode opMode) {
        this.opMode = opMode;
        turner = opMode.hardwareMap.get(Servo.class, "Turner");
        turner.setPosition(FRONT);
    }

    public void runTurner(boolean tooLowToPivot) {
        if (!tooLowToPivot) {
            if (opMode.dpadLeftPressed(opMode.GAMEPAD2)) {
                startMoving(BACK);
            } else if (opMode.dpadRightPressed(opMode.GAMEPAD2)) {
                startMoving(FRONT);
            }
        }

        if (isMoving) {
            currentPosition = Range.clip(currentPosition + turnIncrement, BACK, FRONT);
            turner.setPosition(currentPosition);
            if (currentPosition == BACK || currentPosition == FRONT) {
                isMoving = false;
            }
        }

        opMode.telemetry.addData("Turner Position", currentPosition);
    }

    public void startMoving(double position) {
        if (position == BACK) {
            turnIncrement = BACK_TURN_INCREMENT;
        } else {
            turnIncrement = FRONT_TURN_INCREMENT;
        }
        isMoving = true;
    }

    public void changeTurnerPosition(double position){
        turner.setPosition(position);
    }

    public boolean reachedPosition(boolean tooLowToPivot) {
        if (currentPosition != targetPosition && !tooLowToPivot) {
            turner.setPosition(targetPosition);
            currentPosition = targetPosition;
            timer.reset();
        } else if (targetPosition == currentPosition) {
            if (timer.seconds() > 0.6) {
                return true;
            }
        }
        return false;
    }

    public void setTarget() {
        if (currentPosition == FRONT) {
            targetPosition = BACK;
        } else {
            targetPosition = FRONT;
        }
    }

    public double getPosition() {
        return currentPosition;
    }

    public boolean isMoving() {
        return isMoving;
    }
}
