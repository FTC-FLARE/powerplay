package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.Servo;

public class MM_Collector {
    private final MM_OpMode opMode;
    private Servo grabber = null;

    public static final double CLOSED = 0.07; //0.185 - 0.09
    public static final double OPEN = 1.0;

    private double position = OPEN;

    public MM_Collector(MM_OpMode opMode) {
        this.opMode = opMode;
        grabber = opMode.hardwareMap.get(Servo.class, "Grabber");
        changePosition(OPEN);
    }

    public void runCollector() {
        if (opMode.rightBumperPressed(opMode.GAMEPAD2)) {
            if (getPosition() == OPEN) {
                changePosition(CLOSED);
            } else {
                changePosition(OPEN);
            }
        }
        opMode.telemetry.addData("Collector Position", grabber.getPosition());
    }

    public void changePosition(double position){
        grabber.setPosition(position);
        this.position = position;
    }

    public double getPosition() {
        return position;
    }
}
