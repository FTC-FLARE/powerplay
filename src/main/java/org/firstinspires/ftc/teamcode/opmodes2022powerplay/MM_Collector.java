package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Collector {
    private final MM_OpMode opMode;
    private Servo grabber = null;
    private Servo coneSaver = null;

    public static final double CLOSED = 0.07; //0.185 - 0.09
    public static final double OPEN = 1.0;
    private static final int BACK = 1;
    private static final int FRONT = 0;

    private ElapsedTime timer = new ElapsedTime();
    private double position = OPEN;
    private boolean scored = false;

    private double coneSaverPosition = 1;

    public MM_Collector(MM_OpMode opMode) {
        this.opMode = opMode;
        grabber = opMode.hardwareMap.get(Servo.class, "Grabber");
        coneSaver = opMode.hardwareMap.get(Servo.class,"Conesaver");
        changePosition(OPEN);
        coneSaver.setPosition(BACK);
    }

    public void runCollector() {
        if (opMode.rightBumperPressed(opMode.GAMEPAD2)) {
            if (getPosition() == OPEN) {
                changePosition(CLOSED);
                scored = false;
            } else {
                if (coneSaverPosition == FRONT) {
                    coneSaver.setPosition(BACK);
                    timer.reset();
                    while (timer.seconds() < 0.2) {
                    }
                }
                changePosition(OPEN);
                scored = true;
            }
        }
        opMode.telemetry.addData("Collector Position", grabber.getPosition());
        runConeSaver();
    }

    public void runConeSaver() {
        if (opMode.robot.slide.tooLowToPivot()) {
            coneSaver.setPosition(BACK);
            coneSaverPosition = BACK;
        } else if (!scored) {
           coneSaver.setPosition(FRONT);
           coneSaverPosition = FRONT;
        }
    }

/*            if (opMode.leftBumperPressed(opMode.GAMEPAD2) && !opMode.robot.slide.tooLowToPivot()) {
        if (coneSaverPosition == CONESAVER_CLOSED) {
            coneSaver.setPosition(CONESAVER_OPEN);
            coneSaverPosition = CONESAVER_OPEN;
        } else {
            coneSaver.setPosition(CONESAVER_OPEN);
            coneSaverPosition = CONESAVER_CLOSED;
        }
    }*/
    public void changePosition(double position){
        grabber.setPosition(position);
        this.position = position;
    }

    public double getPosition() {
        return position;
    }
}
