package org.firstinspires.ftc.teamcode.opmodes2022powerplay;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Collector {
    private final MM_OpMode opMode;
    private final MM_Slide slide;
    private MM_Chomper chomper;
    private MM_Conesaver conesaver;

    private ElapsedTime timer = new ElapsedTime();

    public MM_Collector(MM_OpMode opMode, MM_Slide slide) {
        this.opMode = opMode;
        this.slide = slide;

        chomper = new MM_Chomper(opMode);
        conesaver = new MM_Conesaver(opMode, slide);
    }

    public void runCollector() {
        if (opMode.rightBumperPressed(opMode.GAMEPAD2)) {
            autoRunCollector();
        }
        conesaver.runConeSaver(chomper.getPosition());
    }

    public void autoRunCollector(){
        if (chomper.getPosition() == MM_Chomper.OPEN) {
            chomper.changePosition(MM_Chomper.CLOSED);
        } else {
            conesaver.disengage();
            chomper.changePosition(MM_Chomper.OPEN);
        }
    }

    public void engageConesaver(){
        conesaver.engage();
    }
    public void chomp(){
        chomper.changePosition(MM_Chomper.CLOSED);
    }
}
