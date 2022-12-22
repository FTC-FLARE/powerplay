package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.Servo;

public class MM_Turner{
    private final MM_OpMode opMode;
    private Servo turner = null;

    final double FRONT = 0.885;
    final double BACK = 0;

    private double position = FRONT;

    public MM_Turner(MM_OpMode opMode, MM_Slide slide) {
        this.opMode = opMode;
        turner = opMode.hardwareMap.get(Servo.class, "Turner");
        changePosition(FRONT);
    }

    public void runTurner(boolean tooLowtoPivot) {
        if (!tooLowtoPivot) {
            if (opMode.dpadLeftPressed(opMode.GAMEPAD2)) {
                changePosition(BACK);
            } else if (opMode.dpadRightPressed(opMode.GAMEPAD2)) {
                changePosition(FRONT);
            }
        }
        opMode.telemetry.addData("Turner Position", turner.getPosition());
    }

    public void changePosition(double position) {
        turner.setPosition(position);
        this.position = position;
    }

    public double getPosition() {
        return position;
    }
}
