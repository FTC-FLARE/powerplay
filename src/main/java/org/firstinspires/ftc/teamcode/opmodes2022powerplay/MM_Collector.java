package org.firstinspires.ftc.teamcode.opmodes2022powerplay;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Collector {
    private Servo grabber = null;

    private MM_OpMode opMode;

    private final double CLOSED = 0.09; //0.185
    private final int OPEN = 1;

    private double position = OPEN;

    public MM_Collector(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    private void init() {
        grabber = opMode.hardwareMap.get(Servo.class, "Grabber");
        grabber.setPosition(OPEN);
    }

    public void runCollector() {
/*        if (opMode.rightBumperPressed(opMode.GAMEPAD2)) {
            grabber.setPosition(CLOSED);
        } else if (opMode.leftBumperPressed(opMode.GAMEPAD2)) {
            grabber.setPosition(OPEN);
        }*/

        if (opMode.rightBumperPressed(opMode.GAMEPAD2)) {
            if (position == OPEN) {
                grabber.setPosition(CLOSED);
                position = CLOSED;
            } else {
                grabber.setPosition(OPEN);
                position = OPEN;
            }
        }

        opMode.telemetry.addData("Collector Position", grabber.getPosition());
    }
}
