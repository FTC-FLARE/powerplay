package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.Servo;

public class MM_Turner{
    private Servo turner = null;

    private MM_OpMode opMode;
    private MM_Slide slide;

    final double FRONT = 0.885;
    final double BACK = 0;

    public MM_Turner(MM_OpMode opMode, MM_Slide slide) {
        this.opMode = opMode;
        this.slide = slide;
        init();
    }

    private void init() {
        turner = opMode.hardwareMap.get(Servo.class, "Turner");
        turner.setPosition(FRONT);
    }

    public void runTurner() {
        if (!slide.tooLowToPivot()) {
            if (opMode.dpadLeftPressed(opMode.GAMEPAD2)) {
                turner.setPosition(BACK);
            } else if (opMode.dpadRightPressed(opMode.GAMEPAD2)) {
                turner.setPosition(FRONT);
            }
        }

        opMode.telemetry.addData("Turner Position", turner.getPosition());
    }
}
