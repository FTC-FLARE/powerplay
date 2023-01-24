package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MM_Turner{
    private final MM_OpMode opMode;
    private Servo turner = null;

    static final double FRONT = 0.885;
    static final double SIDE = 0.4425;
    static final double BACK = 0;
    static final double FRONT_TURN_INCREMENT = 0.025;
    static final double BACK_TURN_INCREMENT = -0.020;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean isMoving = false;
    private double currentPosition = FRONT;
    private double targetPosition = FRONT;
    private double turnIncrement = BACK_TURN_INCREMENT;

    public MM_Turner(MM_OpMode opMode) {
        this.opMode = opMode;
        turner = opMode.hardwareMap.get(Servo.class, "Turner");
        turner.setPosition(FRONT);
    }

    public void startMoving(double increment) {
        turnIncrement = increment;
        isMoving = true;
    }

    public void checkIfDoneMoving() {
        currentPosition = Range.clip(currentPosition + turnIncrement, BACK, FRONT);
        changePosition(currentPosition);
        if (currentPosition == BACK || currentPosition == FRONT) {
            isMoving = false;
        }
        opMode.telemetry.addData("Turner Position", currentPosition);
    }

    public boolean reachedPosition(boolean tooLowToPivot) {
        if (currentPosition != targetPosition && !tooLowToPivot) {
            changePosition(targetPosition);
            currentPosition = targetPosition;
            timer.reset();
        } else {
            return targetPosition == currentPosition && timer.seconds() > 0.6;
        }

        return false;
    }

    public void changePosition(double position){
        turner.setPosition(position);
    }

    public double getPosition() {
        return currentPosition;
    }

    public boolean isMoving() {
        return isMoving;
    }
}
