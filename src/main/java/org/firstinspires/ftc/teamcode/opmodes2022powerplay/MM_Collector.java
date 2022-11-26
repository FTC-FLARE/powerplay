package org.firstinspires.ftc.teamcode.opmodes2022powerplay;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Collector {
    private Servo grabber = null;

    private MM_OpMode opMode;

    private final int CLOSED = 0;
    private final int OPEN = 1;

    public MM_Collector(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    private void init() {
        grabber = opMode.hardwareMap.get(Servo.class, "Grabber");
        grabber.setPosition(OPEN);
    }

    public void runCollector() {
        if (opMode.rightBumperPressed(opMode.GAMEPAD2)) {
            grabber.setPosition(CLOSED);
        } else if (opMode.leftBumperPressed(opMode.GAMEPAD2)) {
            grabber.setPosition(OPEN);
        }

        opMode.telemetry.addData("Collector Position", grabber.getPosition());
    }
}
