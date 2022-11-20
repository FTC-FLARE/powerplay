package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Collector{
    private MM_OpMode opMode;

    final int CLOSED = 0;
    final int OPEN = 1;

    private Servo grabber = null;

    private int currentPosition = CLOSED;

    public MM_Collector (MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    private void init() {
        grabber = opMode.hardwareMap.get(Servo.class, "Grabber");
        grabber.setPosition(OPEN);
    }

    public void runCollector() {
        if (opMode.leftBumperPressed(opMode.GAMEPAD2)) {
            if (currentPosition == CLOSED) {
                grabber.setPosition(OPEN);
                currentPosition = OPEN;
            } else {
                grabber.setPosition(CLOSED);
                currentPosition = CLOSED;
            }
        }

        opMode.telemetry.addData("Collector Position", grabber.getPosition());
    }
}
