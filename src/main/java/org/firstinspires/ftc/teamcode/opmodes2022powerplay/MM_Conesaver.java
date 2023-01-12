package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Conesaver {
    private final MM_OpMode opMode;
    private MM_Slide slide; // passed to the constructor
    private Servo coneSaver = null;

    public static final int BACK = 1;
    public static final int FRONT = 0;

    private ElapsedTime timer = new ElapsedTime();

    private double coneSaverPosition = 1;

    public MM_Conesaver(MM_OpMode opMode, MM_Slide slide) {
        this.opMode = opMode;
        this.slide = slide;
        coneSaver = opMode.hardwareMap.get(Servo.class,"Conesaver");
        coneSaver.setPosition(BACK);
    }
    public void changePosition(double position){
        coneSaver.setPosition(position);
    }


    void disengage() {
        if (coneSaverPosition == FRONT) {
            changePosition(BACK);
            timer.reset();
            while (opMode.opModeIsActive() && timer.seconds() < 0.2) {
            }
        }
    }
    public void runConeSaver(double chomperPosition) {
        if (slide.tooLowtoConesave()) {
            coneSaver.setPosition(BACK);
            coneSaverPosition = BACK;
        } else if (chomperPosition == MM_Chomper.CLOSED) {
            coneSaver.setPosition(FRONT);
            coneSaverPosition = FRONT;
        }
    }
    public void engage() {
        coneSaver.setPosition(FRONT);
        coneSaverPosition = FRONT;
    }


}
